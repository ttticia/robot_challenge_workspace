#!/usr/bin/env python3
"""MuJoCo cell maze, vendor policy UDP transport and optional ROS sensors."""
import argparse
import math
import os
from pathlib import Path
import socket
import signal
import struct
import time
import json
import mujoco
import mujoco.viewer
import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def rpy(q):
    w, x, y, z = q
    return [math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y)),
            math.asin(np.clip(2*(w*y-z*x), -1, 1)),
            math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))]


def run(args):
    model = mujoco.MjModel.from_xml_path(str(args.model.resolve()))
    model.opt.timestep = .001
    data = mujoco.MjData(model)
    policy = None
    if args.controller:
        from mujoco_policy import Policy
        policy = Policy(model, maximum_tilt_deg=args.maximum_tilt_deg)
        data.qpos[7:19] = policy.default
        data.qpos[2] = .34
    else:
        data.qpos[7:19] = [0, -1.35453, 2.54948]*4
    mujoco.mj_forward(model, data)
    receive = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    receive.bind(('127.0.0.1', 20001))
    receive.setblocking(False)
    send = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sensor = viewer = None
    paused = False
    def key(keycode):
        nonlocal paused
        if keycode == 32:
            paused = not paused
    try:
        if args.ros:
            import rclpy
            from rclpy.signals import SignalHandlerOptions
            from mujoco_sensors import Sensors
            rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
            sensor = Sensors(model, data, rgb=not args.no_rgb, stereo=args.stereo)
        if args.controller:
            if not args.ros:
                raise ValueError('--controller requires --ros')
        if not args.headless:
            viewer = mujoco.viewer.launch_passive(model, data, key_callback=key)
            viewer.cam.lookat[:] = [2., 2., 0]
            viewer.cam.distance, viewer.cam.azimuth, viewer.cam.elevation = 6.25, 90., -65.
        print('MuJoCo maze ready. Space pauses; close window or Ctrl+C exits.', flush=True)
        print('Control: integrated policy via /cmd_vel.' if policy else 'Control: external vendor controller on loopback UDP 20001/30010.', flush=True)
        print('Factory gait presets are not emulated.', flush=True)
        command = np.zeros((5, 12))
        command[0], command[1], command[2] = 20., data.qpos[7:19], 1.
        last_packet = time.monotonic()
        deadline = time.perf_counter()
        step = 0
        single_commander = False
        start = time.monotonic()
        max_tilt = 0.
        minimum_height = float(data.qpos[2])
        wall_contact_steps = 0
        deepest_wall_contact = 0.
        wall_ids = {g.id for g in (model.geom(i) for i in range(model.ngeom)) if g.name.startswith('wall_')}
        while (not viewer or viewer.is_running()) and (not args.seconds or time.monotonic()-start < args.seconds):
            if sensor:
                rclpy.spin_once(sensor, timeout_sec=0)
            if paused:
                if sensor:
                    sensor.cmd[:] = 0
                viewer.sync()
                time.sleep(.01)
                deadline = time.perf_counter()
                continue
            for _ in range(8):
                try:
                    packet = receive.recv(241)
                except BlockingIOError:
                    break
                if len(packet) == 240:
                    proposed = np.frombuffer(packet, dtype='<f4').reshape(5, 12)
                    if np.isfinite(proposed).all():
                        command = proposed.copy()
                        last_packet = time.monotonic()
            torque = command[0]*(command[1]-data.qpos[7:19]) + command[2]*(command[3]-data.qvel[6:18]) + command[4]
            if policy:
                if step % 50 == 0:
                    single_commander = sensor.count_publishers('/cmd_vel') == 1
                fresh = time.monotonic()-sensor.cmd_time < .3 and single_commander
                velocity = sensor.cmd if fresh else np.zeros(3)
                torque = policy.torque(model, data, velocity, step)
            elif time.monotonic()-last_packet > .3:
                torque = -1.5*data.qvel[6:18]
            data.ctrl[:] = np.clip(torque, -35., 35.)
            mujoco.mj_step(model, data)
            step += 1
            if step % 20 == 0:
                max_tilt = max(max_tilt, *map(abs, rpy(data.qpos[3:7])[:2]))
                minimum_height = min(minimum_height, float(data.qpos[2]))
                contacts = [c for c in data.contact if (c.geom1 in wall_ids) != (c.geom2 in wall_ids)]
                if contacts:
                    wall_contact_steps += 1
                    deepest_wall_contact = max(deepest_wall_contact, max(-float(c.dist) for c in contacts))
            payload = [*rpy(data.qpos[3:7]), *data.sensordata[16:19], *data.sensordata[19:22],
                       *data.qpos[7:19], *data.qvel[6:18], *data.ctrl]
            send.sendto(struct.pack('<d45f', data.time, *payload), ('127.0.0.1', 30010))
            if sensor:
                if step % 10 == 0:
                    sensor.fast()
                if step % 100 == 0:
                    sensor.lidar_scan()
                if step % 200 == 0:
                    sensor.images()
            if viewer and step % 33 == 0:
                viewer.sync()
            deadline += .001/args.realtime_factor
            remaining = deadline-time.perf_counter()
            if remaining > 0:
                time.sleep(remaining)
            elif remaining < -.1:
                deadline = time.perf_counter()
        print(f'Completed {data.time:.3f} simulated seconds in {time.monotonic()-start:.3f} wall seconds', flush=True)
        print(f'Final evaluation pose: xyz={data.qpos[:3].tolist()}, rpy={rpy(data.qpos[3:7])}, policy_fault={policy.fault if policy else None}', flush=True)
    finally:
        if 'start' in locals():
            metrics = dict(simulated_seconds=data.time, wall_seconds=time.monotonic()-start,
                maximum_roll_or_pitch_deg=math.degrees(max_tilt), minimum_body_height_m=minimum_height,
                wall_contact_samples=wall_contact_steps, wall_contact_sampling_seconds=.02,
                deepest_wall_contact_m=deepest_wall_contact, policy_fault=policy.fault if policy else None)
            args.metrics.parent.mkdir(parents=True, exist_ok=True)
            args.metrics.write_text(json.dumps(metrics, indent=2)+'\n')
        if viewer:
            viewer.close()
        if sensor:
            sensor.close()
            if rclpy.ok():
                rclpy.shutdown()
        receive.close()
        send.close()


if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal.default_int_handler)
    signal.signal(signal.SIGTERM, signal.default_int_handler)
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', type=Path, default=ROOT/'assets/maze_rough.xml')
    parser.add_argument('--ros', action='store_true')
    parser.add_argument('--controller', action='store_true', help='Run vendor ONNX policy on the physics thread with ROS velocity input')
    parser.add_argument('--no-rgb', action='store_true')
    parser.add_argument('--stereo', action='store_true')
    parser.add_argument('--headless', action='store_true', default=os.getenv('LITE3_HEADLESS') == '1')
    parser.add_argument('--seconds', type=float, default=0)
    parser.add_argument('--realtime-factor', type=float, default=1., help='Target physics speed / wall time')
    parser.add_argument('--metrics', type=Path, default=ROOT/'logs/current/physics_metrics.json')
    parser.add_argument('--maximum-tilt-deg', type=float, default=0., help='Optional policy attitude stop; 0 disables it')
    try:
        run(parser.parse_args())
    except KeyboardInterrupt:
        pass
