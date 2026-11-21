"""The Python environment against the TypeScript one (`tests/golden_env_ts.json`, produced by `BatchEnv` in `sdk/typescript` on the same task, seed and actions),
and its own behaviour on the real wasm."""
import json
from pathlib import Path

import numpy as np
import pytest

from dynova import Batch, BatchEnv, EnvError, EnvSpec, episode_seed, mulberry32
from dynova.envs.reacher import ReacherEnv

GOLD = json.loads((Path(__file__).parent / "golden_env_ts.json").read_text())
CAP, RING = 16, 64


class EndsBothWays(ReacherEnv):
    max_steps = 40

    def terminated(self, world, index, ctx, obs):
        return abs(float(obs[0])) > 0.2


def action(n: int, t: int) -> np.ndarray:
    # Integer arithmetic and one multiplication: no transcendental that two languages could round differently.
    return np.array([(((t * 7 + i * 13) % 11) - 5) * 0.6 for i in range(n)], dtype=np.float32)


def fnv(words, h=0xCBF29CE484222325):
    for x in words:
        h = ((h ^ int(x)) * 0x100000001B3) & 0xFFFFFFFFFFFFFFFF
    return h


def trajectory(env: BatchEnv, steps: int = 130):
    def feed(h, r):
        h = fnv(r.obs.view(np.uint32).ravel(), h)
        h = fnv(r.reward.view(np.uint32).ravel(), h)
        for flags in (r.terminated, r.truncated, r.resetting):
            h = fnv(flags.astype(np.uint32), h)
        return h

    h = feed(0xCBF29CE484222325, env.result)
    first, counts = [], [0, 0, 0]
    for t in range(steps):
        r = env.step(action(env.size, t))
        h = feed(h, r)
        if t < 2:
            first.extend(int(v) for v in r.obs.view(np.uint32).ravel())
        counts = [counts[0] + int(r.terminated.sum()), counts[1] + int(r.truncated.sum()), counts[2] + int(r.resetting.sum())]
    return {"digest": str(h), "terminated": counts[0], "truncated": counts[1], "resets": counts[2], "first": first}


def make(worlds, spec, seed=0, workers=1, cap=CAP, ring=RING):
    batch = Batch(worlds, capacity=cap, ring_capacity=ring, workers=workers)
    return batch, BatchEnv(batch, spec, seed=seed)


@pytest.mark.parametrize("name, spec", [("both_ends", EndsBothWays()), ("default", ReacherEnv())])
@pytest.mark.parametrize("workers", [1, 2, 3])
def test_a_130_step_trajectory_equals_the_typescript_one_bit_for_bit(name, spec, workers):
    batch, env = make(3, spec, seed=5, workers=workers)
    with batch:
        assert trajectory(env) == GOLD[name]


def test_both_ways_of_ending_an_episode_happen_in_the_golden_run():
    assert GOLD["both_ends"]["terminated"] > 0 and GOLD["both_ends"]["truncated"] > 0
    assert GOLD["both_ends"]["resets"] > 0


def test_the_generator_draws_what_the_typescript_one_pins():
    r = mulberry32(1)
    assert [r.next(), r.next(), r.next()] == [0.6270739405881613, 0.002735721180215478, 0.5274470399599522]
    assert mulberry32(0).next() == 0.26642920868471265
    assert mulberry32(4294967295).next() == 0.8964226141106337
    assert [episode_seed(0, 0, 0), episode_seed(0, 1, 0), episode_seed(5, 3, 2)] == [2746606380, 4026859071, 1330923999]
    assert len({episode_seed(0, w, e) for w in range(64) for e in range(32)}) == 64 * 32


def test_range_and_int_stay_inside_their_bounds():
    r = mulberry32(99)
    for _ in range(2000):
        assert -2 <= r.range(-2, 3) < 3
        assert 0 <= r.int(7) < 7


def test_the_first_observation_has_the_target_each_worlds_own_seed_draws():
    batch, env = make(4, ReacherEnv(), seed=11)
    with batch:
        for i in range(4):
            target = np.float32(mulberry32(episode_seed(11, i, 0)).range(-1.2, 1.2))
            assert list(env.result.obs[i]) == [0.0, 0.0, target]
        assert env.result.resetting.all() and not env.result.reward.any()
        assert len(set(env.result.obs[:, 2])) == 4


def test_the_same_seed_repeats_and_another_does_not():
    runs = []
    for seed in (5, 5, 6):
        batch, env = make(3, ReacherEnv(), seed=seed)
        with batch:
            runs.append(trajectory(env, 60))
    assert runs[0] == runs[1] and runs[0] != runs[2]


def test_a_world_has_the_same_history_whatever_the_number_of_worlds_beside_it():
    seen = []
    for worlds in (2, 5):
        batch, env = make(worlds, ReacherEnv(), seed=9)
        with batch:
            rows = []
            for t in range(100):
                r = env.step(action(worlds, t))
                rows.append((r.obs[:2].copy(), r.reward[:2].copy()))
            seen.append(rows)
    for (o1, r1), (o2, r2) in zip(*seen):
        assert np.array_equal(o1, o2) and np.array_equal(r1, r2)


def test_a_reset_returns_to_where_an_episode_starts_and_replays_the_same_run():
    batch, env = make(3, ReacherEnv(), seed=21)
    with batch:
        start = env.result.obs.copy()
        first = trajectory(env, 60)
        assert not np.array_equal(env.result.obs, start)
        assert np.array_equal(env.reset().obs, start)
        assert trajectory(env, 60) == first


def test_a_new_seed_on_reset_changes_the_targets_and_the_same_seed_repeats_them():
    batch, env = make(2, ReacherEnv(), seed=1)
    with batch:
        one = env.reset(7).obs.copy()
        two = env.reset(8).obs.copy()
        assert not np.array_equal(one, two)
        assert np.array_equal(env.reset(7).obs, one)


def test_the_action_on_a_reset_step_is_ignored_and_the_step_after_it_is_ordinary():
    class One(ReacherEnv):
        max_steps = 1

    a, ea = make(1, One(), seed=4)
    b, eb = make(1, One(), seed=4)
    with a, b:
        ea.step([[0]]); eb.step([[0]])
        ra, rb = ea.step([[3]]), eb.step([[-3]])
        assert ra.resetting[0] and np.array_equal(ra.obs, rb.obs) and ra.reward[0] == rb.reward[0] == 0
        assert np.array_equal(ea.step([[1]]).obs, eb.step([[1]]).obs)


def test_terminated_wins_over_truncated_on_one_step():
    class Both(ReacherEnv):
        max_steps = 1

        def terminated(self, *args):
            return True

    batch, env = make(1, Both())
    with batch:
        r = env.step([[0]])
        assert r.terminated[0] and not r.truncated[0]
        assert env.step([[0]]).resetting[0]


def test_results_are_reused_not_reallocated():
    batch, env = make(2, ReacherEnv())
    with batch:
        a = env.step(np.zeros((2, 1)))
        b = env.step(np.zeros((2, 1)))
        assert a is b and a.obs is b.obs


@pytest.mark.parametrize("actions, error, message", [
    (np.zeros(3), ValueError, "expected 2 action values"),
    (np.array([0.0, np.nan]), ValueError, "action 1 \\(world 1\\) is not finite"),
    (np.array([np.inf, 0.0]), ValueError, "action 0 \\(world 0\\) is not finite"),
])
def test_hostile_actions_are_refused_and_the_environment_stays_usable(actions, error, message):
    batch, env = make(2, ReacherEnv())
    with batch:
        with pytest.raises(error, match=message):
            env.step(actions)
        env.step(np.zeros(2))


@pytest.mark.parametrize("bad", [float("nan"), 1.5, -1, 2 ** 32, True])
def test_a_seed_must_be_an_integer_in_range_and_a_bad_one_changes_nothing(bad):
    with Batch(1, capacity=CAP, ring_capacity=RING) as batch:
        with pytest.raises(ValueError, match="seed"):
            BatchEnv(batch, ReacherEnv(), seed=bad)
    batch, env = make(2, ReacherEnv(), seed=3)
    with batch:
        before = env.result.obs.copy()
        with pytest.raises(ValueError, match="seed"):
            env.reset(bad)
        assert np.array_equal(env.result.obs, before)
        env.step(np.zeros(2))


def test_sizes_that_are_not_positive_integers_and_an_empty_batch_are_refused():
    for field in ("obs_size", "action_size", "max_steps"):
        for bad in (0, -1, 1.5, None):
            spec = ReacherEnv()
            setattr(spec, field, bad)
            with Batch(1, capacity=CAP, ring_capacity=RING) as batch:
                with pytest.raises(ValueError, match=field):
                    BatchEnv(batch, spec)

    class Empty:
        worlds = []

    with pytest.raises(ValueError, match="at least one world"):
        BatchEnv(Empty(), ReacherEnv())


def test_a_hook_that_throws_names_its_world_and_leaves_the_environment_unusable():
    class Boom(ReacherEnv):
        def act(self, world, index, ctx, action):
            if index == 1:
                raise RuntimeError("boom")
            super().act(world, index, ctx, action)

    batch, env = make(3, Boom())
    with batch:
        with pytest.raises(EnvError, match="world 1 act: boom"):
            env.step(np.zeros(3))
        with pytest.raises(EnvError, match="unusable after an earlier failure: world 1 act: boom"):
            env.step(np.zeros(3))
        with pytest.raises(EnvError, match="unusable"):
            env.reset()


def test_a_reward_that_throws_or_is_not_finite_breaks_the_environment():
    class BadReward(ReacherEnv):
        def reward(self, world, index, ctx, obs):
            return float("inf") if index == 0 else 0.0

    batch, env = make(2, BadReward())
    with batch:
        with pytest.raises(EnvError, match="world 0 gave a non-finite reward"):
            env.step(np.zeros(2))
        with pytest.raises(EnvError, match="unusable"):
            env.step(np.zeros(2))


def test_a_non_finite_observation_names_the_world_and_index():
    class BadObs(ReacherEnv):
        def observe(self, world, index, ctx, out):
            super().observe(world, index, ctx, out)
            if index == 1:
                out[1] = float("nan")

    with Batch(2, capacity=CAP, ring_capacity=RING) as batch:
        with pytest.raises(EnvError, match="world 1 observed a non-finite value at index 1"):
            BatchEnv(batch, BadObs())


def test_a_build_of_more_commands_than_the_ring_holds_is_checkpointed_as_built_and_unmoved():
    BODIES = 200

    class Many(EnvSpec):
        obs_size, action_size, max_steps = 2, 1, 50

        def build(self, world, index, count):
            world.spawn(0, -5, 0, half=(50, 1, 50), inv_mass=0)
            for k in range(BODIES):
                world.spawn(k * 2, 5, 0, half=(0.4, 0.4, 0.4))

        def act(self, world, index, ctx, action):
            pass

        def observe(self, world, index, ctx, out):
            t = world.read_transforms()
            out[0] = float(np.count_nonzero(t[: BODIES + 1, 6]))
            out[1] = t[1, 1]

        def reward(self, *a):
            return 0.0

        def terminated(self, *a):
            return False

    batch, env = make(2, Many(), cap=256, ring=64)
    with batch:
        assert env.result.obs.tolist() == [[BODIES + 1, 5.0]] * 2
        for _ in range(30):
            env.step(np.zeros(2))
        assert env.result.obs[0, 1] < 4.9
        env.reset()
        assert env.result.obs.tolist() == [[BODIES + 1, 5.0]] * 2


def test_a_hook_that_queues_more_commands_than_the_ring_takes_is_refused():
    class Flood(ReacherEnv):
        def act(self, world, index, ctx, action):
            for _ in range(70):
                world.set_joint_motor(ctx.joint, 1.0, 20.0)

    batch, env = make(1, Flood(), ring=64)
    with batch:
        with pytest.raises(EnvError, match="world 0 queued more commands than its ring of 64 slots"):
            env.step(np.zeros(1))
        with pytest.raises(EnvError, match="unusable"):
            env.step(np.zeros(1))


def test_a_world_without_physics_fails_the_build_because_the_checkpoint_is_refused():
    with Batch(1, capacity=CAP, ring_capacity=RING, physics=False) as batch:
        with pytest.raises(EnvError, match="world 0: \\d+ command\\(s\\) were refused while it was built"):
            BatchEnv(batch, ReacherEnv())


def test_a_reset_refused_because_a_recording_runs_is_an_error_not_a_silent_carry_on():
    class Recording(ReacherEnv):
        def build(self, world, index, count):
            world.record(True)
            return super().build(world, index, count)

    with Batch(1, capacity=CAP, ring_capacity=RING) as batch:
        with pytest.raises(EnvError, match="world 0: \\d+ command\\(s\\) were refused while it was reset"):
            BatchEnv(batch, Recording())


def test_a_body_spawned_mid_episode_vanishes_at_the_reset_and_ids_come_back_clean():
    batch, env = make(1, ReacherEnv(), seed=2)
    with batch:
        world = batch.worlds[0]
        for _ in range(3):
            env.step(np.zeros(1))
        mid = world.spawn(7, 7, 0, half=(0.5, 0.5, 0.5), inv_mass=0)
        env.step(np.zeros(1))
        assert world.read_transforms()[mid.id, 0] == 7
        env.reset()
        assert world.read_transforms()[mid.id, 6] == 0
        again = world.spawn(-4, 2, 0, half=(0.5, 0.5, 0.5), inv_mass=0)
        env.step(np.zeros(1))
        assert again.id == mid.id and list(world.read_transforms()[again.id, :3]) == [-4.0, 2.0, 0.0]


def test_a_failed_world_stops_the_environment_with_the_world_named():
    batch, env = make(3, ReacherEnv(), workers=3)
    with batch:
        batch._groups[1].batch.step_all = lambda: (_ for _ in ()).throw(__import__("dynova").WasmTrap("lost"))
        with pytest.raises(EnvError, match="worlds failed: 1: .*lost"):
            env.step(np.zeros(3))
        with pytest.raises(EnvError, match="unusable"):
            env.step(np.zeros(3))


def test_256_worlds_on_four_workers_equal_the_same_256_worlds_on_one():
    def run(workers):
        batch, env = make(256, ReacherEnv(), seed=3, workers=workers)
        with batch:
            rows = [env.result.obs.copy()]
            for t in range(50):
                rows.append(env.step(action(256, t)).obs.copy())
            return rows

    one, four = run(1), run(4)
    assert all(np.array_equal(a, b) for a, b in zip(one, four))
    assert len({tuple(row) for row in one[-1].tolist()}) > 200, "the worlds differ from each other"


@pytest.mark.parametrize("value, message", [
    (None, "not a number"), ("1", "not a number"), (True, "not a number"), (np.array([1.0]), "not a number"),
    (float("nan"), "non-finite reward"), (float("-inf"), "non-finite reward"),
])
def test_a_reward_that_is_not_a_finite_number_breaks_the_environment_with_the_world_named(value, message):
    class Odd(ReacherEnv):
        def reward(self, world, index, ctx, obs):
            return value

    batch, env = make(1, Odd())
    with batch:
        with pytest.raises(EnvError, match=f"world 0 gave a .*{message}"):
            env.step(np.zeros(1))
        with pytest.raises(EnvError, match="unusable"):
            env.step(np.zeros(1))


def test_a_reward_hook_that_raises_names_the_hook():
    class Raises(ReacherEnv):
        def reward(self, world, index, ctx, obs):
            raise RuntimeError("no reward today")

    batch, env = make(1, Raises())
    with batch:
        with pytest.raises(EnvError, match="world 0 reward: no reward today"):
            env.step(np.zeros(1))


@pytest.mark.parametrize("value", [1, 0, None, "False", float("nan"), np.array([True, False])])
def test_a_terminated_hook_must_return_a_bool(value):
    class Odd(ReacherEnv):
        def terminated(self, world, index, ctx, obs):
            return value

    batch, env = make(1, Odd())
    with batch:
        with pytest.raises(EnvError, match="world 0 terminated hook must return a bool"):
            env.step(np.zeros(1))


def test_a_terminated_hook_that_raises_names_the_hook_and_numpy_bools_are_accepted():
    class Raises(ReacherEnv):
        def terminated(self, world, index, ctx, obs):
            raise RuntimeError("cannot tell")

    class NumpyBool(ReacherEnv):
        def terminated(self, world, index, ctx, obs):
            return np.bool_(False)

    batch, env = make(1, Raises())
    with batch:
        with pytest.raises(EnvError, match="world 0 terminated: cannot tell"):
            env.step(np.zeros(1))
    batch, env = make(1, NumpyBool())
    with batch:
        env.step(np.zeros(1))


def test_a_non_finite_observation_during_a_step_breaks_the_environment():
    class Late(ReacherEnv):
        calls = 0

        def observe(self, world, index, ctx, out):
            super().observe(world, index, ctx, out)
            Late.calls += 1
            if Late.calls > 2:
                out[2] = float("nan")

    Late.calls = 0
    batch, env = make(2, Late())
    with batch:
        with pytest.raises(EnvError, match="world 0 observed a non-finite value at index 2"):
            env.step(np.zeros(2))
        with pytest.raises(EnvError, match="unusable"):
            env.step(np.zeros(2))


def test_a_hook_cannot_start_a_step_or_reset_inside_a_step():
    holder = {}

    class Reentrant(ReacherEnv):
        def act(self, world, index, ctx, action):
            holder["env"].step(np.zeros(2))

    batch, env = make(2, Reentrant())
    holder["env"] = env
    with batch:
        with pytest.raises(EnvError, match="world 0 act: a reset or step is already running"):
            env.step(np.zeros(2))


def test_a_bad_call_changes_nothing_so_the_next_step_equals_a_fresh_environments():
    a, ea = make(2, ReacherEnv(), seed=4)
    b, eb = make(2, ReacherEnv(), seed=4)
    with a, b:
        for bad in (np.zeros(3), np.array([0.0, np.nan]), np.zeros((1, 2)), [["x"], ["y"]]):
            with pytest.raises(ValueError):
                ea.step(bad)
            assert all(w.pending_commands == 0 for w in a.worlds) and ea._broken is None and not ea._busy
        ra, rb = ea.step(np.ones(2)), eb.step(np.ones(2))
        assert np.array_equal(ra.obs, rb.obs) and np.array_equal(ra.reward, rb.reward)


def test_numpy_integers_are_accepted_for_the_seed_and_the_sizes():
    class Sized(ReacherEnv):
        obs_size, action_size, max_steps = np.int64(3), np.int64(1), np.int64(200)

    a, ea = make(1, Sized(), seed=np.uint32(5))
    b, eb = make(1, ReacherEnv(), seed=5)
    with a, b:
        assert np.array_equal(ea.result.obs, eb.result.obs)


def test_a_closed_batch_stops_the_environment_with_an_env_error():
    class Idle(ReacherEnv):
        def act(self, world, index, ctx, action):
            pass

    batch, env = make(2, Idle())
    batch.close()
    with pytest.raises(EnvError, match="the batch cannot step: the batch is closed"):
        env.step(np.zeros(2))
    batch, env = make(2, ReacherEnv())
    batch.close()
    with pytest.raises(EnvError, match="world 0 act: .*closed"):
        env.step(np.zeros(2))


def test_a_failed_reset_breaks_the_environment():
    class Fussy(ReacherEnv):
        calls = 0

        def reset(self, world, index, ctx, rng):
            Fussy.calls += 1
            if Fussy.calls > 1:
                raise RuntimeError("no more resets")
            super().reset(world, index, ctx, rng)

    Fussy.calls = 0
    batch, env = make(1, Fussy())
    with batch:
        with pytest.raises(EnvError, match="world 0 reset: no more resets"):
            env.reset()
        with pytest.raises(EnvError, match="unusable"):
            env.step(np.zeros(1))
