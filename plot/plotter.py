#! /usr/bin/env python3
import sys, getopt
import rosbag
import numpy as np
import matplotlib.pyplot as plt
import pdb


def plot_pos(bag_name):
    bag = rosbag.Bag(bag_name)

    t0 = None
    t0_d = None
    ts = []
    t_ds = []
    x = []
    y = []
    z = []
    x_d = []
    y_d = []
    z_d = []

    for (topic, msg, timestamp) in bag.read_messages():

        if topic == '/position':
            t = msg.header.stamp.secs + msg.header.stamp.nsecs * 10**-9
            if t0 == None:
                t0 = t
            ts.append(t - t0)
            x.append(float(msg.x))
            y.append(float(msg.y))
            z.append(float(msg.z))

        if topic == '/position_ref':
            t_d = msg.header.stamp.secs + msg.header.stamp.nsecs * 10**-9
            if t0_d == None:
                t0_d = t_d
            t_ds.append(t_d - t0_d)
            x_d.append(float(msg.x))
            y_d.append(float(msg.y))
            z_d.append(float(msg.z))

    plt.figure(3)
    plt.subplot(3,1,1)
    plt.plot(ts, x, t_ds, x_d)
    plt.ylabel('x [m]')
    plt.subplot(3,1,2)
    plt.plot(ts, y, t_ds, y_d)
    plt.ylabel('y [m]')
    plt.subplot(3,1,3)
    plt.plot(ts, y, t_ds, y_d)
    plt.ylabel('y [m]')
    plt.suptitle("Position tracking")
    plt.show()

def plot_attitude(bag_name):
    bag = rosbag.Bag(bag_name)

    t0 = None
    t0_r = None
    t0_cmd = None
    ts = []
    ts_r = []
    ts_cmd = []
    roll = []
    pitch = []
    roll_r = []
    pitch_r = []
    roll_cmd = []
    pitch_cmd = []

    for (topic, msg, timestamp) in bag.read_messages():
        if topic == '/attitude_euler':
            t = msg.header.stamp.secs + msg.header.stamp.nsecs * 10**-9
            if t0 == None:
                t0 = t
            ts.append(t - t0)
            roll.append(float(msg.x))
            pitch.append(float(msg.y))

        if topic == '/attitude_ref_euler':
            t_r = msg.header.stamp.secs + msg.header.stamp.nsecs * 10**-9
            if t0_r == None:
                t0_r = t_r
            ts_r.append(t_r - t0_r)
            roll_r.append(float(msg.x))
            pitch_r.append(float(msg.y))

        if topic == '/attitude_cmd_traj_euler':
            t_cmd = msg.header.stamp.secs + msg.header.stamp.nsecs * 10**-9
            if t0_cmd == None:
                t0_cmd = t_cmd
            ts_cmd.append(t_cmd - t0_cmd)
            roll_cmd.append(float(msg.x))
            pitch_cmd.append(float(msg.y))

    plt.figure(1)
    plt.subplot(2,1,1)
    plt.plot(ts, roll)
    plt.plot(ts_r, roll_r)
    plt.plot(ts_cmd, roll_cmd)
    plt.ylabel('roll [rad]')
    #plt.ylim(-0.1, 0.1)

    plt.subplot(2,1,2)
    plt.plot(ts, pitch)
    plt.plot(ts_r, pitch_r)
    plt.plot(ts_cmd, pitch_cmd)
    plt.ylabel('pitch [rad]')
    #plt.ylim(-0.1, 0.1)
    plt.legend(["Actual", "Reference", "Command"])
    plt.suptitle("Attitude tracking")
    plt.show()



def plot_pos_error(bag_name):
    bag = rosbag.Bag(bag_name)

    t0 = None
    ts = []
    x = []
    y = []
    z = []

    for (topic, msg, timestamp) in bag.read_messages():
        if topic == '/position_error':
            t = msg.header.stamp.secs + msg.header.stamp.nsecs * 10**-9
            if t0 == None:
                t0 = t
            ts.append(t - t0)
            x.append(float(msg.x))
            y.append(float(msg.y))
            z.append(float(msg.z))

    zero_ref = np.zeros(len(ts))

    plt.figure(2)
    plt.subplot(3,1,1)
    plt.plot(ts, x, ts, zero_ref)
    plt.ylabel('x position [m]')
    plt.subplot(3,1,2)
    plt.plot(ts, y, ts, zero_ref)
    plt.ylabel('y position [m]')
    plt.subplot(3,1,3)
    plt.plot(ts, z, ts, zero_ref)
    plt.ylabel('z position [m]')
    plt.suptitle("Position tracking error")
    plt.show()

def main(argv):
    bag_name = "data.bag"

    try:
        opts, args = getopt.getopt(argv, "pae", ["pos=", "att=", "err="])
    except getopt.GetoptError:
        print("Unknown command line args")
        sys.exit(2)
    for opt, arg in opts:
        if opt in ("-p", "--pos"):
            plot_pos(bag_name)
        if opt in ("-a", "--att"):
            plot_attitude(bag_name)
        if opt in ("-e", "--err"):
            plot_pos_error(bag_name)

if __name__ == '__main__':
    main(sys.argv[1:])

