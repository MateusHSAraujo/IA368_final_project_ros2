#!/usr/bin/env python3
"""Replace the class 2D laser sweep of /myRobot with a fast depth-camera laser.

The class laser (child script on /myRobot/LaserScanner_2D) moves a joint and fires a
proximity sensor 452 times per simulation step in Lua. That alone slows the simulation to
~0.47x real time in p3dx_nav2.ttt and ~0.24x in scene_house.ttt.

This tool adds two perspective depth vision sensors (92 degrees each, facing +45 and -45
degrees) under /myRobot/LaserScanner_2D and swaps the laser script for one that renders
them on the GPU and resamples the depth buffers into the SAME output as before:
452 ranges from -90 degrees in 0.4 degree steps, misses = range_max + 1 (11.0), written to
the 'signal.distSignal' float array property of /myRobot. The ROS bridge is unchanged.

The scene is modified in memory only: save it in CoppeliaSim afterwards to keep it.

Usage (from WSL; CoppeliaSim on Windows with the scene open, simulation stopped):
    python3 -u scripts/coppelia/install_fast_laser.py            # install (idempotent)
    python3 -u scripts/coppelia/install_fast_laser.py --uninstall
    python3 -u scripts/coppelia/install_fast_laser.py --host 172.23.224.1
"""

import argparse
import math
import os
import sys

import zmq
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'ros2_ws', 'src', 'vr_robot_pkg'))
from vr_robot_pkg.network import detect_windows_host_ip  # noqa: E402

LASER_PATH = '/myRobot/LaserScanner_2D'
SENSOR_NAMES = ('LaserDepthLeft', 'LaserDepthRight')
SENSOR_YAW = (math.pi / 4, -math.pi / 4)
SENSOR_FOV = math.radians(92.0)
SENSOR_WIDTH = 512
NEAR_CLIP = 0.08   # m; clips the scanner body (0.11 m cube around the sensors)
FAR_CLIP = 10.0    # m; along the optical axis
MARKER = '-- fast-depth-laser v2'

FAST_LASER_SCRIPT = MARKER + r'''
-- Fast 2D laser for /myRobot: two 92-degree depth vision sensors rendered on the GPU,
-- installed by install_fast_laser.py (IA368 final project, ROS 2 side).
-- Output is identical in format to the original sweep script: 452 ranges starting at
-- -90 degrees in 0.4 degree steps, misses = rangeMax + 1, written to the
-- 'signal.distSignal' property of /myRobot. The original script text is kept in the
-- 'customData.originalLaserScript' property of this object.
sim = require'sim'

function sysCall_init()
    robotHandle = sim.getObject('/myRobot')
    sensors = {sim.getObject('/myRobot/LaserScanner_2D/LaserDepthLeft'),
               sim.getObject('/myRobot/LaserScanner_2D/LaserDepthRight')}
    sensorYaw = {math.pi / 4, -math.pi / 4}
    rangeMax = 10
    samples = 452
    angleMin = -math.pi / 2
    angleStep = math.pi / 450
    local res = sim.getVisionSensorRes(sensors[1])
    width = res[1]
    farClip = sim.getObjectFloatParam(sensors[1], sim.visionfloatparam_far_clipping)
    local tanHalf = math.tan(sim.getObjectFloatParam(sensors[1], sim.visionfloatparam_perspective_angle) / 2)
    -- Lookup table: for every output sample, the sensor, the depth column and 1/cos(offset).
    lutSensor, lutColumn, lutInvCos = {}, {}, {}
    for i = 1, samples do
        local a = angleMin + (i - 1) * angleStep
        local s = (a >= 0) and 1 or 2
        local off = a - sensorYaw[s]
        local u = math.tan(off) / tanHalf            -- +1 = left edge of the image
        local col = math.floor((1 - u) * width / 2)  -- column 0 = left edge
        lutSensor[i] = s
        lutColumn[i] = math.min(math.max(col, 0), width - 1) + 1
        lutInvCos[i] = 1 / math.cos(off)
    end
end

function sysCall_sensing()
    local depth = {}
    for k = 1, 2 do
        sim.handleVisionSensor(sensors[k])
        local buf = sim.getVisionSensorDepth(sensors[k], 1)  -- depth in meters
        depth[k] = sim.unpackFloatTable(buf)
    end
    local distance = {}
    local missAt = farClip * 0.999
    for i = 1, samples do
        local z = depth[lutSensor[i]][lutColumn[i]]
        local r = rangeMax + 1
        if z < missAt then
            r = z * lutInvCos[i]
            if r > rangeMax then r = rangeMax + 1 end
        end
        distance[i] = r
    end
    sim.setFloatArrayProperty(robotHandle, 'signal.distSignal', distance)
    lastScan = distance
    scanSeq = scanSeq + 1
end

-- Batched access for the ROS bridge. While simulating, the ZMQ Remote API answers only
-- ~2 requests per simulation step, so the bridge makes ONE call per cycle:
--   sim.callScriptFunction('remoteExchange', <this script>, left, right, wantScan, wantCamera, jpegQuality)
-- left/right: wheel joint target velocities (rad/s), or nil to leave them unchanged.
-- Returns {t, pose={x,y,z,qx,qy,qz,qw}, lin={vx,vy,vz}, ang={wx,wy,wz}, scanSeq,
--          scan=(452 ranges, if wantScan), jpeg=(JPEG buffer, if wantCamera), camRes={w,h}}.
scanSeq = 0
function remoteExchange(left, right, wantScan, wantCamera, jpegQuality)
    if not groundTruth then
        groundTruth = sim.getObject('/myRobot/Ground_truth')
        leftMotor = sim.getObject('/myRobot/leftMotor')
        rightMotor = sim.getObject('/myRobot/rightMotor')
        camera = sim.getObject('/myRobot/frontCamera', {noError = true})
    end
    if left and right then
        sim.setJointTargetVelocity(leftMotor, left)
        sim.setJointTargetVelocity(rightMotor, right)
    end
    local lin, ang = sim.getObjectVelocity(groundTruth)
    local out = {t = sim.getSimulationTime(), pose = sim.getObjectPose(groundTruth, -1),
                 lin = lin, ang = ang, scanSeq = scanSeq}
    if wantScan and lastScan then out.scan = lastScan end
    if wantCamera and camera ~= -1 then
        sim.handleVisionSensor(camera)
        local img, res = sim.getVisionSensorImg(camera)
        out.camRes = res
        out.jpeg = sim.saveImage(img, res, 0, '.jpg', jpegQuality or 70)
    end
    return out
end
'''


def connect(host):
    client = RemoteAPIClient(host=host, port=23000)
    client.socket.setsockopt(zmq.RCVTIMEO, 20000)
    return client.require('sim')


def sensor_matrix(yaw):
    """Vision sensor frame (x left, y up, z forward) rotated by yaw about the scanner z axis."""
    c, s = math.cos(yaw), math.sin(yaw)
    # columns: x_s = left of the view direction, y_s = up, z_s = view direction
    return [-s, 0.0, c, 0.0,
            c, 0.0, s, 0.0,
            0.0, 1.0, 0.0, 0.0]


def install(sim):
    laser = sim.getObject(LASER_PATH)
    script = sim.getScript(sim.scripttype_childscript, laser)
    text = sim.getScriptStringParam(script, sim.scriptstringparam_text)
    if not text.startswith('-- fast-depth-laser'):
        sim.writeCustomStringData(laser, 'originalLaserScript', text)
    ref = sim.getObject(LASER_PATH + '/Laserscanner_ref')
    for name, yaw in zip(SENSOR_NAMES, SENSOR_YAW):
        existing = sim.getObject(f'{LASER_PATH}/{name}', {'noError': True})
        if existing != -1:
            sim.removeObjects([existing])
        # options: bit0 explicit handling, bit1 perspective, bit2 hide the sensor body
        h = sim.createVisionSensor(1 | 2 | 4, [SENSOR_WIDTH, 1, 0, 0],
                                   [NEAR_CLIP, FAR_CLIP, SENSOR_FOV, 0.01, 0, 0, 0, 0, 0, 0, 0])
        sim.setObjectAlias(h, name)
        sim.setObjectParent(h, laser, True)
        sim.setObjectMatrix(h, sensor_matrix(yaw), ref)
    sim.setScriptStringParam(script, sim.scriptstringparam_text, FAST_LASER_SCRIPT)
    print('Fast depth laser installed (save the scene to keep it).')


def uninstall(sim):
    laser = sim.getObject(LASER_PATH)
    script = sim.getScript(sim.scripttype_childscript, laser)
    original = sim.readCustomStringData(laser, 'originalLaserScript')
    if original:
        sim.setScriptStringParam(script, sim.scriptstringparam_text, original)
        sim.writeCustomStringData(laser, 'originalLaserScript', '')
    for name in SENSOR_NAMES:
        h = sim.getObject(f'{LASER_PATH}/{name}', {'noError': True})
        if h != -1:
            sim.removeObjects([h])
    print('Original laser restored.' if original else 'No original script stored; sensors removed.')


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--host', default=None, help='CoppeliaSim host (default: detect the Windows host)')
    parser.add_argument('--uninstall', action='store_true')
    args = parser.parse_args()
    sim = connect(args.host or detect_windows_host_ip() or '127.0.0.1')
    if sim.getSimulationState() != sim.simulation_stopped:
        sys.exit('Stop the simulation first.')
    uninstall(sim) if args.uninstall else install(sim)


if __name__ == '__main__':
    main()
