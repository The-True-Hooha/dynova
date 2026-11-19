use crate::command_ring::Command;
use crate::runtime::RuntimeSnapshot;

pub const KEYFRAME_INTERVAL: u32 = 60;
/// A burst of commands forces an early keyframe so replaying from any keyframe stays bounded.
pub const MAX_SEGMENT_COMMANDS: usize = 4096;
pub const MAX_KEYFRAME_BYTES: usize = 64 << 20;

pub struct Keyframe {
    pub tick: u32,
    pub snapshot: RuntimeSnapshot,
}

/// Keyframes plus the commands applied between them. The log holds `(tick, command)` where `tick` is the
/// tick count before the step that applied the command, so replaying keyframe k to tick T applies every
/// entry with `k.tick <= key < T`, each followed by a physics step. Both vectors stay sorted by tick.
pub struct Recorder {
    keyframes: Vec<Keyframe>,
    log: Vec<(u32, Command)>,
    end: u32,
    since_keyframe: usize,
    bytes: usize,
    budget: usize,
}

impl Recorder {
    /// Recording starts at the end of the step that received `Record`, so there is no keyframe until then.
    pub fn new() -> Self {
        Self::with_budget(MAX_KEYFRAME_BYTES)
    }

    pub fn with_budget(budget: usize) -> Self {
        Self { keyframes: Vec::new(), log: Vec::new(), end: 0, since_keyframe: 0, bytes: 0, budget }
    }

    #[cfg(test)]
    pub(crate) fn budget_for_test(&self) -> usize {
        self.budget
    }

    pub fn is_armed(&self) -> bool {
        self.keyframes.is_empty()
    }

    pub fn start(&self) -> u32 {
        self.keyframes.first().map_or(0, |k| k.tick)
    }

    pub fn end(&self) -> u32 {
        self.end
    }

    pub fn keyframe_count(&self) -> usize {
        self.keyframes.len()
    }

    pub fn log_len(&self) -> usize {
        self.log.len()
    }

    /// Only commands that change simulation state belong in the log. Hull uploads and growth persist outside the
    /// snapshot, so the caller must log a command only once the runtime accepted it: a command rejected for a
    /// missing hull or a too-small capacity would be accepted by a replay that runs after the upload or the grow.
    pub fn loggable(cmd: &Command) -> bool {
        matches!(
            cmd,
            Command::Spawn { .. } | Command::Destroy { .. } | Command::Force { .. } | Command::Teleport { .. } | Command::Gravity { .. }
                | Command::JointCreate { .. } | Command::JointDestroy { .. } | Command::JointMotor { .. } | Command::KinematicBatch { .. } | Command::ParticleChain { .. } | Command::ParticleCloth { .. } | Command::ParticleAttach { .. } | Command::ParticleDetach { .. }
        )
    }

    pub fn push_command(&mut self, tick: u32, cmd: &Command) {
        self.log.push((tick, cmd.clone()));
        self.since_keyframe += 1;
    }

    pub fn wants_keyframe(&self, tick: u32) -> bool {
        self.is_armed() || tick.is_multiple_of(KEYFRAME_INTERVAL) || self.since_keyframe >= MAX_SEGMENT_COMMANDS
    }

    /// Commands applied on a paused world pile up at the head tick without a step to put a keyframe after them.
    pub fn wants_bake(&self) -> bool {
        self.since_keyframe >= MAX_SEGMENT_COMMANDS
    }

    /// Keyframe taken on a paused world: its state already includes the commands logged at `tick`, so they leave the
    /// log; replaying them on top of this keyframe would apply them twice.
    pub fn bake_pending(&mut self, tick: u32, snapshot: RuntimeSnapshot) {
        let cut = self.log.partition_point(|(t, _)| *t < tick);
        self.log.truncate(cut);
        self.push_keyframe(tick, snapshot);
    }

    pub fn push_keyframe(&mut self, tick: u32, snapshot: RuntimeSnapshot) {
        self.bytes += snapshot.approx_bytes();
        self.keyframes.push(Keyframe { tick, snapshot });
        self.since_keyframe = 0;
        self.end = tick;
        while self.total_bytes() > self.budget && self.keyframes.len() > 1 {
            self.bytes -= self.keyframes.remove(0).snapshot.approx_bytes();
            let first = self.start();
            let cut = self.log.partition_point(|(t, _)| *t < first);
            self.log.drain(..cut);
        }
    }

    fn total_bytes(&self) -> usize {
        self.bytes + self.log.len() * std::mem::size_of::<(u32, Command)>()
    }

    pub fn advance_end(&mut self, tick: u32) {
        self.end = tick;
    }

    /// A live step taken after a seek replaces the recorded future.
    pub fn truncate_to(&mut self, tick: u32) {
        let keep = self.keyframes.partition_point(|k| k.tick <= tick);
        for k in self.keyframes.drain(keep..) {
            self.bytes -= k.snapshot.approx_bytes();
        }
        let cut = self.log.partition_point(|(t, _)| *t < tick);
        self.log.truncate(cut);
        let last = self.keyframes.last().map_or(0, |k| k.tick);
        self.since_keyframe = self.log.len() - self.log.partition_point(|(t, _)| *t < last);
        self.end = tick;
    }

    /// The nearest keyframe at or before `target` and the commands to replay on top of it, or None when
    /// `target` lies outside the recorded range.
    pub fn plan(&self, target: u32) -> Option<(&Keyframe, Vec<(u32, Command)>)> {
        if self.is_armed() || target < self.start() || target > self.end {
            return None;
        }
        let k = &self.keyframes[self.keyframes.partition_point(|k| k.tick <= target) - 1];
        let from = self.log.partition_point(|(t, _)| *t < k.tick);
        let to = self.log.partition_point(|(t, _)| *t < target);
        Some((k, self.log[from..to].to_vec()))
    }
}

impl Default for Recorder {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::runtime::Runtime;

    fn snap() -> RuntimeSnapshot {
        let mut rt = Runtime::new(8, 8);
        rt.enable_physics();
        rt.capture().unwrap()
    }

    fn force(id: u32) -> Command {
        Command::Force { id, generation: 0, x: 1.0, y: 0.0, z: 0.0 }
    }

    fn recorder_with(ticks: &[u32]) -> Recorder {
        let mut r = Recorder::new();
        r.push_keyframe(0, snap());
        for &t in ticks {
            r.push_command(t, &force(t));
        }
        r
    }

    #[test]
    fn only_state_changing_commands_are_loggable() {
        let spawn_body = crate::physics::BodyDesc::box_at([0.0; 3], [0.5; 3], 1.0, 0.0);
        assert!(Recorder::loggable(&Command::Spawn { id: 0, generation: 0, body: spawn_body }));
        assert!(Recorder::loggable(&Command::Destroy { id: 0, generation: 0 }));
        assert!(Recorder::loggable(&force(0)));
        assert!(Recorder::loggable(&Command::Teleport { id: 0, generation: 0, x: 0.0, y: 0.0, z: 0.0 }));
        assert!(Recorder::loggable(&Command::Gravity { x: 0.0, y: 0.0, z: 0.0 }));
        let desc = crate::physics::JointDesc {
            kind: crate::physics::JOINT_BALL, a: 0, b: 1, anchor: [0.0; 3], axis: [0.0; 3], limits: None, motor_vel: 0.0, motor_max: 0.0,
        };
        assert!(Recorder::loggable(&Command::JointCreate { id: 0, gen_a: 0, gen_b: 0, desc }));
        assert!(Recorder::loggable(&Command::JointDestroy { id: 0 }));
        assert!(Recorder::loggable(&Command::JointMotor { id: 0, vel: 0.0, max: 0.0 }));
        for c in [
            Command::Grow { capacity: 8 }, Command::Inspect { on: true }, Command::Record { on: true }, Command::Seek { tick: 1 },
            Command::HullVerts { hull: 0, count: 1, verts: [[0.0; 3]; 4] }, Command::HullCommit { hull: 0, count: 4 },
        ] {
            assert!(!Recorder::loggable(&c), "{c:?}");
        }
    }

    #[test]
    fn a_new_recorder_is_armed_until_its_first_keyframe_and_refuses_plans() {
        let mut r = Recorder::new();
        assert!(r.is_armed() && r.wants_keyframe(7));
        assert!(r.plan(0).is_none());
        r.push_keyframe(5, snap());
        assert!(!r.is_armed());
        assert_eq!((r.start(), r.end(), r.keyframe_count()), (5, 5, 1));
    }

    #[test]
    fn keyframes_are_wanted_every_interval_and_after_a_command_burst() {
        let mut r = recorder_with(&[]);
        assert!(!r.wants_keyframe(1) && !r.wants_keyframe(KEYFRAME_INTERVAL - 1));
        assert!(r.wants_keyframe(KEYFRAME_INTERVAL));
        for _ in 0..MAX_SEGMENT_COMMANDS - 1 { r.push_command(1, &force(0)); }
        assert!(!r.wants_keyframe(1));
        r.push_command(1, &force(0));
        assert!(r.wants_keyframe(1));
        r.push_keyframe(2, snap());
        assert!(!r.wants_keyframe(3));
    }

    #[test]
    fn plan_picks_the_nearest_earlier_keyframe_and_only_the_commands_between() {
        let mut r = recorder_with(&[1, 2, 3]);
        r.push_keyframe(4, snap());
        for t in [4, 5, 6] { r.push_command(t, &force(t)); }
        r.advance_end(7);
        let ticks = |p: Option<(&Keyframe, Vec<(u32, Command)>)>| p.map(|(k, c)| (k.tick, c.iter().map(|(t, _)| *t).collect::<Vec<_>>()));
        assert_eq!(ticks(r.plan(0)), Some((0, vec![])));
        assert_eq!(ticks(r.plan(3)), Some((0, vec![1, 2])));
        assert_eq!(ticks(r.plan(4)), Some((4, vec![])));
        assert_eq!(ticks(r.plan(6)), Some((4, vec![4, 5])));
        assert_eq!(ticks(r.plan(7)), Some((4, vec![4, 5, 6])));
        assert!(r.plan(8).is_none());
    }

    #[test]
    fn truncating_drops_the_future_keyframes_commands_and_resets_the_segment_count() {
        let mut r = recorder_with(&[1, 2]);
        r.push_keyframe(3, snap());
        for t in [3, 4, 5] { r.push_command(t, &force(t)); }
        r.advance_end(6);
        r.truncate_to(4);
        assert_eq!((r.end(), r.keyframe_count(), r.log_len()), (4, 2, 3));
        assert_eq!(r.plan(4).unwrap().1.len(), 1, "only the command at tick 3 lies between keyframe 3 and tick 4");
        r.truncate_to(2);
        assert_eq!((r.end(), r.keyframe_count(), r.log_len()), (2, 1, 1));
        for _ in 0..MAX_SEGMENT_COMMANDS - 1 { r.push_command(2, &force(0)); }
        assert!(r.wants_keyframe(2), "the segment count was recomputed from the surviving log");
    }

    #[test]
    fn a_keyframe_at_the_truncation_tick_survives_and_its_commands_do_not() {
        let mut r = recorder_with(&[1]);
        r.push_keyframe(2, snap());
        r.push_command(2, &force(2));
        r.advance_end(3);
        r.truncate_to(2);
        assert_eq!((r.keyframe_count(), r.log_len(), r.end()), (2, 1, 2));
    }

    #[test]
    fn logged_commands_count_against_the_byte_budget() {
        let one = snap().approx_bytes();
        let entry = std::mem::size_of::<(u32, Command)>();
        let mut r = Recorder::with_budget(one * 3 + entry * 10);
        r.push_keyframe(0, snap());
        r.push_keyframe(1, snap());
        for _ in 0..20 { r.push_command(1, &force(0)); }
        r.push_keyframe(2, snap());
        assert_eq!(r.start(), 1, "the oldest keyframe went because the log pushed the total over budget");
        assert_eq!(r.keyframe_count(), 2);
    }

    #[test]
    fn the_byte_budget_evicts_the_oldest_keyframes_and_their_commands() {
        let one = snap().approx_bytes();
        let mut r = Recorder::with_budget(one * 2 + std::mem::size_of::<(u32, Command)>() * 4);
        r.push_keyframe(0, snap());
        r.push_command(0, &force(0));
        r.push_keyframe(1, snap());
        r.push_command(1, &force(1));
        r.push_keyframe(2, snap());
        assert_eq!((r.start(), r.keyframe_count(), r.log_len()), (1, 2, 1));
        assert!(r.plan(0).is_none());
        assert!(r.plan(1).is_some());
        let mut tiny = Recorder::with_budget(0);
        tiny.push_keyframe(0, snap());
        tiny.push_keyframe(1, snap());
        assert_eq!(tiny.keyframe_count(), 1, "the newest keyframe is always kept");
    }

    #[test]
    fn baking_pending_commands_into_a_keyframe_removes_them_from_the_log() {
        let mut r = recorder_with(&[1, 2]);
        r.advance_end(3);
        r.push_command(3, &force(3));
        r.push_command(3, &force(3));
        r.bake_pending(3, snap());
        assert_eq!((r.keyframe_count(), r.log_len(), r.end()), (2, 2, 3));
        let (k, cmds) = r.plan(3).unwrap();
        assert_eq!((k.tick, cmds.len()), (3, 0), "seeking the baked tick lands on the keyframe, no replay");
        assert!(!r.wants_bake());
    }

    #[test]
    fn a_bake_is_wanted_only_after_a_full_segment() {
        let mut r = recorder_with(&[]);
        for _ in 0..MAX_SEGMENT_COMMANDS - 1 { r.push_command(0, &force(0)); }
        assert!(!r.wants_bake());
        r.push_command(0, &force(0));
        assert!(r.wants_bake());
    }

    #[test]
    fn a_total_exactly_at_the_budget_is_kept() {
        let one = snap().approx_bytes();
        let mut r = Recorder::with_budget(one * 2);
        r.push_keyframe(0, snap());
        r.push_keyframe(1, snap());
        assert_eq!(r.keyframe_count(), 2);
        r.push_keyframe(2, snap());
        assert_eq!((r.start(), r.keyframe_count()), (1, 2));
    }

    #[test]
    fn truncating_gives_back_the_dropped_keyframes_bytes() {
        let one = snap().approx_bytes();
        let mut r = Recorder::with_budget(one * 2);
        r.push_keyframe(0, snap());
        r.push_keyframe(1, snap());
        r.truncate_to(0);
        r.push_keyframe(1, snap());
        assert_eq!(r.keyframe_count(), 2, "the forked-away keyframe no longer counts against the budget");
    }
}
