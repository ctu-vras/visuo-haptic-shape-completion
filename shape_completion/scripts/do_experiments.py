#!/usr/bin/env python2
"""
Script for easier experiment running
@author Lukas Rustler
"""
from subprocess import Popen, call
import json
import datetime
import os
import time
import argparse
import sys
import signal


def signal_handler(signal, frame):
    """
    Handler for killing the program
    @param signal: signal type
    @type signal: signal
    @param frame:
    @type frame:
    @return:
    @rtype:
    """
    print("Killing on user request")
    call("pkill -f ros", shell=True)  # Kill all ROS related things

    sys.exit(0)


def prepare_parser():
    """
    Reads argument and return them in correct form
    @return:
    @rtype:
    """
    arg_parser = argparse.ArgumentParser(
        description="Experimentator"
    )
    arg_parser.add_argument(
        "--setup_file",
        "-s",
        dest="setup_file_name",
        required=True,
        help="Path to setup_file_name"
    )
    arg_parser.add_argument(
        "--Logs output folder",
        "-l",
        dest="logs_folder",
        required=False,
        default=None,
        help="Folder where to save experiments logs"
    )

    arg_parser.add_argument(
        "--detection_type",
        "-d",
        dest="detection_type",
        required=False,
        default="cusum",
        help="What collision detection algo to use: cusum, threshold, openrave_threshold, openrave_cusum"
    )

    arg_parser.add_argument(
        "--interactive",
        "-i",
        dest="interactive",
        action="store_true",
        required=False,
        default=False,
        help="When interactive, RVIZ is started and user is prompted if everything runs fine"
    )

    arg_parser.add_argument(
        "--real_setup",
        "-r",
        dest="real_setup",
        action="store_true",
        required=False,
        default=False,
        help="If to run real_setup"
    )


    args = arg_parser.parse_args()
    return args.setup_file_name, args.logs_folder, args.detection_type, args.interactive, args.real_setup


if __name__ == "__main__":
    # Kill ros after CTRL+C signal
    signal.signal(signal.SIGINT, signal_handler)

    # Open setup file
    setup_file_name, logs_folder, detection_type, interactive, real_setup = prepare_parser()
    with open(setup_file_name, "r") as setup_file:
        exp_setup = json.load(setup_file)

    # If logs_folder not specified, create one besides the setups directory
    if logs_folder is None:
        logs_folder = os.path.join(os.path.dirname(os.path.dirname(setup_file_name)), 'logs')

    if not os.path.exists(logs_folder):
        os.makedirs(logs_folder)

    # Help variables
    folders = ["meshes", "pcd", "npy", "npz", "plots", "rosbags", "binvoxes", "grasps"]
    file_dir = os.path.dirname(os.path.abspath(__file__))
    start_time = str(datetime.datetime.now()).replace(".", "-").replace(" ", "-").replace(":", "-")

    # Write header into .log file
    log_file = open(os.path.join(logs_folder, start_time)+".log", "w")
    log_file.write("(object, number of reconstructions, number of touches, number of repetitions)\n\n")
    log_file.close()

    for object, rec_num, touches, repetitions in zip(exp_setup["objects"], exp_setup["reconstructions"], exp_setup["touches"], exp_setup["repetitions"]):
        for repetition in range(repetitions):
            if not real_setup:
                # Run RVIZ, mujoco etc.
                #/bin/bash -i -c shape_rec_p2 && 
                cmd = "run_simulation "+object+" '[0.6,0,0.15]' 'true' 'true'"
            else:
                #/bin/bash -i -c shape_rec_p2 && 
                cmd = "roslaunch shape_completion real.launch printed_finger:='true' object_name:='"+object+"'"
                print(cmd)
            rviz = Popen(cmd, shell=True)#, stdout=PIPE)

            if not interactive:
                # Wait long enough to give everything time to boot
                time.sleep(30)
            else:
                # Wait for user input confirming that everything is okay
                while True:
                    answer = raw_input("Everything fine?")
                    if answer.lower() in ["y", "yes"]:
                        break
                    else:
                        call("pkill -f ros", shell=True)
                        time.sleep(15)
                        rviz = Popen(cmd, shell=True)  # , stdout=PIPE)

            # Run the experiment and wait for end
            #/bin/bash -i -c shape_rec_p2 && 
            cmd = "rosrun shape_completion main.py -r " + str(rec_num) + " -t" + str(touches) +" -d " + detection_type
            main_loop = call(cmd, shell=True)

            # Kill all ros related to have clean starting point next iteration
            call("pkill -f ros", shell=True)

            # Move all files into timestamped folders
            timestamp_ = str(datetime.datetime.now())
            timestamp = timestamp_.replace(".", "-").replace(" ", "-").replace(":", "-")
            for folder in folders:
                folder_path = os.path.join(file_dir, '../data', folder, object)
                Popen("mv "+folder_path+" "+folder_path+"_"+timestamp, shell=True)
            time.sleep(15)

            # Save info into .log file
            log_file = open(os.path.join(logs_folder, start_time)+".log", "a")
            log_file.write(str(repetition+1)+". repetition of ("+",".join([str(object), str(rec_num), str(touches), str(repetitions)])+") completed at: " + timestamp_+"\n")
            log_file.close()



