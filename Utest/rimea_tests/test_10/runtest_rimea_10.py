#!/usr/bin/env python
import os
import sys
utestdir = os.path.abspath(os.path.dirname(os.path.dirname(sys.path[0])))
from sys import *
sys.path.append(utestdir)
from JPSRunTest import JPSRunTestDriver
from utils import *




def run_rimea_test10(inifile, trajfile):
    fps, N, traj = parse_file(trajfile)
    failure = False
    group_1 = [1, 2, 3, 4, 5, 6, 7, 12, 13, 14, 15, 16, 17, 18, 19] # Haupt tuer
    group_2 = [8, 9, 10, 11, 20, 21, 22, 23] # sekundaer tuer
    e1 = [11, 6, 7.2] # y, x1, x2 |--> haupt
    e2 = [18, 5, 6]   # x, y1, y2 |--> sekundaer
    for ped in group_1:
        traj1 = traj[traj[:, 0] == ped]
        if not PassedLineY(traj1, e1):
            logging.critical("ped %d did not exit from Exit (%1.2f, %1.2f) | (%1.2f, %1.2f)"%
                             (ped, e1[1], e1[0], e1[2], e1[0]))
            failure = True
        else:
            logging.info("ped %d  exits from Exit (%1.2f, %1.2f) | (%1.2f, %1.2f)"%
                         (ped, e1[1], e1[0], e1[2], e1[0]))

    for ped in group_2:
        traj1 = traj[traj[:, 0] == ped]
        if not PassedLineX(traj1, e2):
            logging.critical("ped %d did not exit from Exit (%1.2f, %1.2f) | (%1.2f, %1.2f)"%
                             (ped, e2[0], e2[1], e2[0], e2[2]))
            failure = True
        else:
            logging.info("ped %d  exits from Exit (%1.2f, %1.2f) | (%1.2f, %1.2f)"%
                         (ped, e2[0], e2[1], e2[0], e2[2]))

    if failure:
        logging.critical("%s exists with failure!"%argv[0])
        exit(FAILURE)



if __name__ == "__main__":
    test = JPSRunTestDriver(10, argv0=argv[0], testdir=sys.path[0], utestdir=utestdir)
    test.run_test(testfunction=run_rimea_test10)
    logging.info("%s exits with SUCCESS" % (argv[0]))
    exit(SUCCESS)







