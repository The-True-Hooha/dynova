import json
import math
from pathlib import Path

import numpy as np
import pytest

from dynova import Batch, RobotError, forward_kinematics, parse_urdf, spawn_robot
from dynova.robot import compose, qrot

ASSETS = Path(__file__).resolve().parents[2] / "assets" / "robots"
GOLD = json.loads((Path(__file__).parent / "golden_robot_ts.json").read_text())


def asset(name: str) -> str:
    return (ASSETS / f"{name}.urdf").read_text()


def one_world(capacity=64):
    batch = Batch(1, capacity=capacity, ring_capacity=1024)
    return batch, batch.worlds[0]


@pytest.mark.parametrize("case", GOLD["cases"], ids=lambda c: c["asset"])
def test_forward_kinematics_of_the_shipped_assets_equals_the_frames_typescript_pinned(case):
    desc = parse_urdf(asset(case["asset"]))
    frames = forward_kinematics(desc, case["values"])
    assert sorted(case["frames"]) == sorted(l.name for l in desc.links)
    for name, want in case["frames"].items():
        got = [*frames[name][0], *frames[name][1]]
        assert max(abs(a - b) for a, b in zip(got, want)) < 1e-12, name


def test_every_shipped_asset_loads_without_warnings_other_than_the_known_ones():
    for name in ("arm", "quadruped", "humanoid"):
        desc = parse_urdf(asset(name))
        assert len(desc.links) > 1
        assert [w for w in desc.warnings if "cylinder is a capsule" not in w] == []
    assert len(parse_urdf(asset("quadruped")).joints) == 12


@pytest.mark.parametrize("text,message", [
    ("<model/>", "not <robot>"),
    ("<robot/>", "no links"),
    ("<robot><link/></robot>", "no name"),
    ('<robot><link name="a"/><link name="a"/></robot>', "defined twice"),
    ('<robot><link name="a"/><link name="b"/></robot>', "one root"),
    ("<!DOCTYPE x><robot/>", "DOCTYPE"),
    ('<robot><link name="a"><collision><geometry><sphere radius="0x10"/></geometry></collision></link></robot>', "finite numbers"),
    ('<robot><link name="a"><collision><geometry><box size="1 0 1"/></geometry></collision></link></robot>', "must be positive"),
    ('<robot><link name="a"/><link name="b"/><joint name="j" type="floating"><parent link="a"/><child link="b"/></joint></robot>', "not supported"),
    ('<robot><link name="a"/><link name="b"/><joint name="j" type="revolute"><parent link="a"/><child link="b"/></joint></robot>', "needs a <limit>"),
    ('<robot><link name="a"/><link name="b"/><joint name="j" type="fixed"><parent link="a"/><child link="zz"/></joint></robot>', "unknown child"),
    ("<robot><link", "not well-formed"),
])
def test_bad_descriptions_are_refused_with_a_message(text, message):
    with pytest.raises(RobotError, match=message):
        parse_urdf(text)


def test_an_imported_arm_driven_to_joint_values_takes_the_pose_forward_kinematics_gives():
    desc = parse_urdf(asset("arm"))
    batch, world = one_world()
    with batch:
        world.set_gravity(0, 0, 0)
        bot = spawn_robot(world, desc, fixed_base=True, up_axis="z")
        target = {"yaw": 0.7, "elbow": -0.6, "slide": 0.12}
        for _ in range(900):
            states = {int(s["joint"]): s for s in world.read_joint_states()}
            for name, joint in bot.joints.items():
                position, velocity = (float(states[joint.id]["position"]), float(states[joint.id]["velocity"])) if joint.id in states else (0.0, 0.0)
                speed = float(np.clip(6 * (target[name] - position) - 0.5 * velocity, -2, 2))
                world.set_joint_motor(joint, speed, 200)
            assert batch.step() == []
        fk = forward_kinematics(desc, target)
        tr = world.read_transforms()
        for l in desc.links:
            want = compose(fk[l.name], l.shape.frame)
            got = tr[bot.links[l.name].id]
            assert np.linalg.norm(got[:3] - np.array(want[0])) < 0.01, l.name


def stand(name, ticks, position, max_force, gains=(10.0, 0.3)):
    desc = parse_urdf(asset(name))
    batch, world = one_world()
    with batch:
        world.spawn(0, -1, 0, half=(100, 1, 50), inv_mass=0, friction=1.0)
        bot = spawn_robot(world, desc, position=position, friction=1.0)
        driven = {n: j for n, j in bot.joints.items()}
        pelvis = bot.links["trunk" if name == "quadruped" else "pelvis"]
        tilt = 0.0
        for _ in range(ticks):
            states = {int(s["joint"]): s for s in world.read_joint_states()}
            for n, joint in driven.items():
                position, velocity = (float(states[joint.id]["position"]), float(states[joint.id]["velocity"])) if joint.id in states else (0.0, 0.0)
                world.set_joint_motor(joint, float(np.clip(gains[0] * -position - gains[1] * velocity, -6, 6)), max_force)
            assert batch.step() == []
            row = world.read_transforms()[pelvis.id]
            up = qrot(tuple(float(v) for v in row[3:7]), (0, 0, 1))
            tilt = max(tilt, math.acos(max(-1.0, min(1.0, up[1]))))
        row = world.read_transforms()[pelvis.id]
        return tilt, row[:3].copy()


def test_the_quadruped_stands_from_python():
    tilt, pos = stand("quadruped", 3000, (0, 0.545, 0), 40)
    assert tilt < 0.05 and 0.48 < pos[1] < 0.53 and math.hypot(pos[0], pos[2]) < 0.05


def test_the_humanoid_stands_from_python():
    tilt, pos = stand("humanoid", 3000, (0, 0.94, 0), 150)
    assert tilt < 0.15 and 0.9 < pos[1] < 0.97 and math.hypot(pos[0], pos[2]) < 0.1


def test_a_spawn_that_cannot_finish_destroys_what_it_made_and_raises():
    links = "".join(f'<link name="l{i}"><inertial><mass value="1"/></inertial><collision><geometry><sphere radius="0.1"/></geometry></collision></link>' for i in range(20))
    joints = "".join(f'<joint name="j{i}" type="fixed"><parent link="l{i}"/><child link="l{i + 1}"/><origin xyz="1 0 0"/></joint>' for i in range(19))
    desc = parse_urdf(f'<robot name="r">{links}{joints}</robot>')
    batch = Batch(1, capacity=8, ring_capacity=128)
    with batch:
        with pytest.raises(Exception):
            spawn_robot(batch.worlds[0], desc, up_axis="z")
        batch.step()
        assert not np.any(batch.worlds[0].read_transforms()[:, 6] != 0)


def test_hostile_input_is_a_robot_error_and_the_limits_match_the_typescript_reader():
    for bad in (b"<robot/>", '<robot><link name="a\ud800"/></robot>', '<robot xmlns="http://x"/>', "<robot>" + "<a>" * 70 + "</a>" * 70 + "</robot>", "<robot>" + "<a/>" * 200_001 + "</robot>", " " * ((16 << 20) + 1)):
        with pytest.raises(RobotError):
            parse_urdf(bad)
    parse_urdf('<robot><link name="a"/>' + "<x>" * 60 + "</x>" * 60 + "</robot>")
    with pytest.raises(RobotError, match="finite numbers"):
        parse_urdf('<robot><link name="a"><collision><geometry><sphere radius="\u0661"/></geometry></collision></link></robot>')
    # Only the first <inertial> counts, as in the TypeScript reader.
    d = parse_urdf('<robot><link name="a"><inertial/><inertial><mass value="3"/></inertial></link></robot>')
    assert d.links[0].mass is None


def test_a_prismatic_limit_between_one_thousand_and_the_engine_bound_is_kept_and_bad_numbers_are_refused():
    text = '<robot><link name="a"><inertial><mass value="1"/></inertial><collision><geometry><box size="1 1 1"/></geometry></collision></link><link name="b"><inertial><mass value="1"/></inertial><collision><geometry><box size="1 1 1"/></geometry></collision></link><joint name="j" type="prismatic"><parent link="a"/><child link="b"/><axis xyz="1 0 0"/><limit lower="-5000" upper="5000"/></joint></robot>'
    batch, world = one_world(8)
    with batch:
        assert not [w for w in spawn_robot(world, parse_urdf(text), fixed_base=True, up_axis="z").warnings if "clamped" in w]
        with pytest.raises(ValueError, match="finite"):
            spawn_robot(world, parse_urdf(text), default_mass=float("nan"))
        with pytest.raises(ValueError, match="finite"):
            spawn_robot(world, parse_urdf(text), friction=float("inf"))


def test_clamps_warns_and_makes_anchors_like_typescript():
    text = '<robot name="w"><link name="a"><inertial><mass value="1e-7"/></inertial><collision><geometry><cylinder radius="0.1" length="0.05"/></geometry></collision></link></robot>'
    batch, world = one_world(8)
    with batch:
        world.set_gravity(0, 0, 0)
        bot = spawn_robot(world, parse_urdf(text), position=(1, 2, 3), up_axis="z")
        assert any("half length 0 was clamped to 0.001" in w for w in bot.warnings)
        assert any("mass 1e-07 was clamped to 1e-06" in w for w in bot.warnings)
        batch.step()
        assert world.read_transforms()[bot.links["a"].id][:3].tolist() == [1, 2, 3]
    hinged = '<robot name="p"><link name="w"/><link name="b"><inertial><mass value="1"/></inertial><collision><geometry><sphere radius="0.1"/></geometry></collision></link><joint name="j" type="continuous"><parent link="w"/><child link="b"/><origin xyz="0 0 1"/><axis xyz="0 1 0"/></joint></robot>'
    batch, world = one_world(8)
    with batch:
        bot = spawn_robot(world, parse_urdf(hinged), up_axis="z")
        assert len(bot.anchors) == 1 and any("static anchor" in w for w in bot.warnings)
