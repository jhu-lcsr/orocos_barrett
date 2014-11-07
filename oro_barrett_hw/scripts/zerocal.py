#!/usr/bin/env python

from __future__ import print_function

import sys
import os
import argparse
import yaml
import select

import rospy

import sensor_msgs.msg

def yaml_load_pc(filename):
    """yaml loading preserving block comments

    returns a list of tuples (comments, yaml-data)
    """
    blocks = []

    with open(filename) as f:
        for line in f.readlines():
            # check for line comment
            if line.find('#') == 0:
                if len(blocks) == 0 or blocks[-1][1]:
                    # start a new comment block
                    blocks.append([line, ''])
                else:
                    # append the comment to the existing comment block
                    blocks[-1][0] += line
            else:
                if len(blocks) == 0:
                    # start a new data block
                    blocks.append([None, line])
                else:
                    # append the data to the new data block
                    blocks[-1][1] += line

    return [[c, yaml.load(d)] for c, d in blocks]

def yaml_dump_pc(blocks, filename):
    with open(filename, 'w') as f:
        for b, d in blocks:
            f.write(b)
            yaml.safe_dump(d, f)

global resolver_states
resolver_states = None

def resolver_states_cb(msg):
    global resolver_states
    resolver_states = msg

def main():

    rospy.init_node('wam_zerocal')

    myargv = rospy.myargv(argv=sys.argv)

    parser = argparse.ArgumentParser(description='Calibrate the zero offset for a WAM arm')
    parser.add_argument('file_path', metavar='FILE_PATH', type=str, nargs=1,
                        help='the path to the YAML configuration file')

    args = parser.parse_args(args=myargv[1:])
    file_path = args.file_path[0]

    # check if config file exists
    if os.path.exists(file_path):
        print("Initializing with existing YAML file: %s" % file_path)
        # open existing config file
        blocks = yaml_load_pc(file_path)
    print("Hit enter / return to save the resolver offset, ctrl-c to exit.")

    # subscribe to resolver states
    resolver_sub = rospy.Subscriber('wam/resolver_states', sensor_msgs.msg.JointState, resolver_states_cb)

    f = '%2.4f'
    timeout=0.1

    r = rospy.Rate(1)
    pos = None
    save = False
    global resolver_states
    while not rospy.is_shutdown():
        # print the resolver state
        if resolver_states:
            pos = resolver_states.position
            print(('resolvers: [ '+', '.join([f]*len(pos))+' ]') % pos, end='\r')
        else:
            print('Waiting for resolver state message...', end='\r')
        sys.stdout.flush()

        # process stdin
        try:
            rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        except select.error as err:
            break

        if rlist and pos:
            s = sys.stdin.readline().rstrip()
            print('')
            save = raw_input('Save resolver state in %s? [Y/n]: ' % file_path).rstrip().lower() in ['','y']
            break

    if save:
        for c, d in blocks:
            if 'home_resolver_offset' in d:
                d['home_resolver_offset'] = pos

        yaml_dump_pc(blocks, file_path)
        print('Saved resolver state in %s.' % file_path)
    else:
        print('No files changed.')

    return 0

if __name__ == '__main__':
    main()
