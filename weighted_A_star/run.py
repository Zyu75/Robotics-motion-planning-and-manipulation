#!/usr/bin/env python

import argparse, numpy, openravepy, time

from HerbRobot import HerbRobot
from HerbEnvironment import HerbEnvironment
from SimpleEnvironment import SimpleEnvironment
from AStarPlanner import AStarPlanner
import time


def main(robot, planning_env, planner):

    raw_input('Press any key to begin planning')

    if robot == 'simple':
        source_config = numpy.ones(args.dimension)*0.1
        target_config = numpy.ones(args.dimension)*0.9
        start_time = time.time()
        plan = planner.Plan(source_config, target_config)
        print("--- %s seconds ---" % (time.time() - start_time))
    else:
        source_config = numpy.array(robot.GetCurrentConfiguration())
        target_config = numpy.array([ 4.6, -1.76, 0.00, 1.96, -1.15, 0.87, -1.43])
        start_time = time.time()
        plan = planner.Plan(source_config, target_config)

        print("--- %s seconds ---" % (time.time() - start_time))
        traj = robot.ConvertPlanToTrajectory(plan)
        raw_input('Press any key to execute trajectory')
        robot.ExecuteTrajectory(traj)
    import IPython
    IPython.embed()


if __name__ == "__main__":

    #start_time = time.time()

    parser = argparse.ArgumentParser(description='script for testing planners')

    parser.add_argument('-r', '--robot', type=str, default='simple',
                        help='The robot to load (herb or simple)')
    parser.add_argument('-p', '--planner', type=str, default='astar',
                        help='The planner to run: astar')
    parser.add_argument('--resolution', type=float, default=0.1,
                        help='Set the resolution of the grid (default: 0.1)')
    parser.add_argument('--dimension', type=int, default=2,
                        help='Set the dimension of the space (default: 2)')
    parser.add_argument('-d', '--debug', action='store_true',
                        help='Enable debug logging')
    parser.add_argument('-m', '--manip', type=str, default='right',
                        help='The manipulator to plan with (right or left) - only applicable if robot is of type herb')

    args = parser.parse_args()
    #start_time = time.time()
    # First setup the environment and the robot
    if args.robot == 'herb':
        openravepy.RaveInitialize(True, level=openravepy.DebugLevel.Info)
        openravepy.misc.InitOpenRAVELogging()

        if args.debug:
            openravepy.RaveSetDebugLevel(openravepy.DebugLevel.Debug)

        env = openravepy.Environment()
        env.SetViewer('qtcoin')
        env.GetViewer().SetName('Homework 1 Viewer')
        robot = HerbRobot(env, args.manip)
        planning_env = HerbEnvironment(robot, args.resolution)

    elif args.robot == 'simple':
        robot = 'simple'
        planning_env = SimpleEnvironment(0.025, args.dimension)

    else:
        print 'Unknown robot option: %s' % args.robot
        exit(0)

    planner = AStarPlanner(planning_env)

    main(robot, planning_env, planner)

    import IPython
    IPython.embed()
