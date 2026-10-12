#!/bin/sh
# Rebuild MaGaRM from the base assembly: steps 1-5, final printable parts, URDF, MuJoCo model, viewer model.
# Needs uv (https://docs.astral.sh/uv/). Intermediate assemblies (100-200 MB) go to cad/build/ (not in git).
set -e
cd "$(dirname "$0")"
B=../build; M=../motors
PY="uv run -q --python 3.12 --with build123d python -I"
$PY step1_J1.py ../base/base_assembly.step $M/DM-JH17-2EC.stp $B/step1
$PY step2_J2.py $B/step1/magarm_step1_J1.step $M/DM-JH17-2EC.stp $M/DM-J4340P-2EC.stp $B/step2
$PY step3_J4.py $B/step2/magarm_step2_J2.step $M/DM-J4340-2EC.stp $B/step3
$PY step4_gripper.py $B/step3/magarm_step3_J4.step $M/DM-H3510.stp $B/step4
$PY step5_logo.py $B/step4/magarm_step4_gripper.step $B/step5
uv run -q --python 3.12 --with build123d --with trimesh --with networkx python export_parts.py $B/step5/magarm_step5_logo.step ../parts
uv run -q --python 3.12 --with build123d --with numpy --with trimesh python -I make_urdf.py $B/step5/magarm_step5_logo.step ../../urdf ../../sim
uv run -q --python 3.12 --with mujoco --with numpy python -I ../../sim/make_mjcf.py
uv run -q --python 3.12 --with build123d --with trimesh --with networkx python make_viewer.py $B/step5/magarm_step5_logo.step ../../viewer 3.5
