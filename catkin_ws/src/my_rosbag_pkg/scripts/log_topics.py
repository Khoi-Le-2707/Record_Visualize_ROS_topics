#! /usr/bin/env python3
import rospy, os
import subprocess, shlex, psutil
import plot_topics
import global_

def log_topics():
    rospy.init_node("log_topics")
    rospy.loginfo("Node log_topics has been started")
    rate = rospy.Rate(1)

    topic1 = rospy.get_param("/log_topics/topic_1")
    topic2 = rospy.get_param("/log_topics/topic_2")
    topic3 = rospy.get_param("/log_topics/topic_3")
    rospy.loginfo("passed parameter for 'log_topics' node: %s %s %s",topic1,topic2,topic3)

    # Create the new directory (name ="datetime") in "bagfiles" directory 
    os.makedirs(global_.new_directory)
    rospy.loginfo("New directory created: %s", global_.new_directory)   

    #rosbag record
    command = "rosbag record -o "+ global_.new_directory+ "/ --duration=9 " +topic1+" "+topic2+" "+topic3
    command = shlex.split(command) #split a command as argument to subprocess.Popen() function
    rosbag_proc = subprocess.Popen(command) #open a subprocess to run command

    rospy.sleep(10)   #wait 10 seconds for rosbag record
    for proc in psutil.process_iter():
        if "record" in proc.name() and set(command[2:]).issubset(proc.cmdline()): #The command line this process has been called with as a list of strings
            rospy.loginfo("Subprocess killed")
            proc.send_signal(subprocess.signal.SIGINT) # interrupt signal and is raised when press Ctrl + C 
    
    rosbag_proc.send_signal(subprocess.signal.SIGINT)

    #remove the directory if nothing was recorded (because no topic was passed as argument)
    # Check if the directory exists and is empty
    if os.path.exists(global_.new_directory) and not os.listdir(global_.new_directory):
        # Remove the directory
        os.rmdir(global_.new_directory)
        rospy.logerr("You must specify a topic name")
   
    
if __name__== "__main__":
    try:
        log_topics()
        plot_topics.visualize()
    except rospy.ROSInitException:
        pass
