#! /usr/bin/env python3

# Module for plot
import bagpy
from bagpy import bagreader
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

import os
import glob
import shutil
import global_
import rospy
import rostopic
import rosbag


def visualize():
    # Get a list of all bag files in the directory
    bag_files = glob.glob(os.path.join(global_.new_directory, "*.bag"))
    # Get the path to the newest bag file for bagreader
    newest_bag_file = bag_files[0] if bag_files else None

    b = bagreader(newest_bag_file)  # create data folder

    for topic in b.topics:
        # return tuple (topic_type,real topic name, a function to evaluate something )
        topic_type = rostopic._get_topic_type(topic)
        if topic_type[0] == "geometry_msgs/TwistStamped":
            plot_twist(b, global_.new_directory, topic_type)
        elif topic_type[0] == "geometry_msgs/PoseStamped":
            plot_roboter_pose(b, global_.new_directory, topic_type)
        elif topic_type[0] == "sensor_msgs/JointState":
            plot_joint_current(b, global_.new_directory, topic_type)
            plot_joint_states(b, global_.new_directory, topic_type)


def plot_twist(b, bag_dir, topic_type):
    msg = b.message_by_topic(topic_type[1])  # csv file
    df = pd.read_csv(msg)  # data frame

    x = df["Time"].to_numpy()
    x = x-x[0]  # abziehen erstes Element, um Nano Sekunde in Sekunde umzuwandeln
    y1 = df["twist.linear.x"]
    y2 = df["twist.linear.y"]
    y3 = df["twist.linear.z"]
    y4 = df["twist.angular.x"]
    y5 = df["twist.angular.y"]
    y6 = df["twist.angular.z"]
    plt.xlabel("Time [s]")
    plt.ylabel("Value")
    plt.plot(x, y1.to_numpy(), label="twist.linear.x")
    plt.plot(x, y2.to_numpy(), label="twist.linear.y")
    plt.plot(x, y3.to_numpy(), label="twist.linear.z")
    plt.plot(x, y4.to_numpy(), label="twist.angular.x")
    plt.plot(x, y5.to_numpy(), label="twist.angular.y")
    plt.plot(x, y6.to_numpy(), label="twist.angular.z")
    plt.legend()
    plt.title("Twist")
    plt.savefig(os.path.join(bag_dir, "plot_Twist.pdf"))
    plt.savefig(os.path.join(bag_dir, "plot_Twist.png"))
    plt.show()


def plot_roboter_pose(b, bag_dir, topic_type):
    msg = b.message_by_topic(topic_type[1])  # csv file
    df = pd.read_csv(msg)  # data frame

    x = df["Time"].to_numpy()
    x = x-x[0]  # abziehen erstes Element, um Nano Sekunde in Sekunde umzuwandeln
    y1 = df["pose.position.x"]
    y2 = df["pose.position.y"]
    y3 = df["pose.position.z"]
    y4 = df["pose.orientation.x"]
    y5 = df["pose.orientation.y"]
    y6 = df["pose.orientation.z"]
    y7 = df["pose.orientation.w"]
    plt.xlabel("Time [s]")
    plt.ylabel("Value")
    plt.plot(x, y1.to_numpy(), label="pose.position.x")
    plt.plot(x, y2.to_numpy(), label="pose.position.y")
    plt.plot(x, y3.to_numpy(), label="pose.position.z")
    plt.plot(x, y4.to_numpy(), label="pose.orientation.x")
    plt.plot(x, y5.to_numpy(), label="pose.orientation.y")
    plt.plot(x, y6.to_numpy(), label="pose.orientation.z")
    plt.plot(x, y7.to_numpy(), label="pose.orientation.w")
    plt.legend()
    plt.title("Roboter_Pose")
    plt.savefig(os.path.join(bag_dir, "plot_Roboter_Pose.pdf"))
    plt.savefig(os.path.join(bag_dir, "plot_Roboter_Pose.png"))
    plt.show()


def plot_joint_current(b, bag_dir, topic_type):
    msg = b.message_by_topic(topic_type[1])  # csv file
    df = pd.read_csv(msg)  # data frame

    x = df["Time"].to_numpy()
    x = x-x[0]  # abziehen erstes Element, um Nano Sekunde in Sekunde umzuwandeln
    y1 = df["effort_0"]
    y2 = df["effort_1"]
    y3 = df["effort_2"]
    y4 = df["effort_3"]
    y5 = df["effort_4"]
    y6 = df["effort_5"]
    plt.xlabel("Time [s]")
    plt.ylabel("Value")
    plt.plot(x, y1.to_numpy(), label="effort_0")
    plt.plot(x, y2.to_numpy(), label="effort_1")
    plt.plot(x, y3.to_numpy(), label="effort_2")
    plt.plot(x, y4.to_numpy(), label="effort_3")
    plt.plot(x, y5.to_numpy(), label="effort_4")
    plt.plot(x, y6.to_numpy(), label="effort_5")
    plt.legend()
    plt.title("Joint_Current")
    plt.savefig(os.path.join(bag_dir, "plot_Joint_Current.pdf"))
    plt.savefig(os.path.join(bag_dir, "plot_Joint_Current.png"))
    plt.show()


def plot_joint_states(b, bag_dir, topic_type):
    msg = b.message_by_topic(topic_type[1])  # csv file
    df = pd.read_csv(msg)  # data frame

    x = df["Time"].to_numpy()
    x = x-x[0]  # abziehen erstes Element, um Nano Sekunde in Sekunde umzuwandeln
    y1 = df["velocity_0"]
    y2 = df["velocity_1"]
    y3 = df["velocity_2"]
    y4 = df["velocity_3"]
    plt.xlabel("Time [s]")
    plt.ylabel("Value")
    plt.plot(x, y1.to_numpy(), label="velocity_0")
    plt.plot(x, y2.to_numpy(), label="velocity_1")
    plt.plot(x, y3.to_numpy(), label="velocity_2")
    plt.plot(x, y4.to_numpy(), label="velocity_3")
    plt.legend()
    plt.title("Joint_States")
    plt.savefig(os.path.join(bag_dir, "plot_Joint_States.pdf"))
    plt.savefig(os.path.join(bag_dir, "plot_Joint_States.png"))
    plt.show()
