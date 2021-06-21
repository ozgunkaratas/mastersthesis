import sys
import argparse
from fnmatch import fnmatchcase
from rosbag import Bag

def merge():
    '''
    merge xsens hl imu and hl cam bags
    ''' 
    total_included_count = 0
    total_skipped_count = 0
    inputBags = ["hl_imu.bag", "xsens.bag", "hl_cam.bag"]

    with Bag("merged.bag", 'w') as o: 
        for ifile in inputBags:
            matchedtopics = []
            included_count = 0
            skipped_count = 0
            with Bag(ifile, 'r') as ib:
                for topic, msg, t in ib:
                    if any(fnmatchcase(topic, pattern) for pattern in "*"):
                        if not topic in matchedtopics:
                            matchedtopics.append(topic)
                        o.write(topic, msg, t)
                        included_count += 1
                    else:
                        skipped_count += 1
            total_included_count += included_count
            total_skipped_count += skipped_count
