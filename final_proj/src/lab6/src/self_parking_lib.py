#!/usr/bin/env python
import tf
import rospy
import sys
import math
import numpy as np
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Transform, Vector3, Twist
import exp_quat_func as eqf
import ar_tag_subs as ats 
from operator import add
import time
from std_msgs.msg import String,Header,Int32,Float32,Bool

def cal_trans_curr(listener, ar_tags, ar_tags_sel, trans_old, trans_curr):
    # modify trans_old and trans_curr
    # ar_tags: all ar_tags, dict
    # ar_tags_sel: selectd ar_tags number, tuple
    Nar = len(ar_tags_sel)
    j = 0
    while j != Nar:
        i = ar_tags_sel[j] #ar_tag number in dict
        try:            
            # listener needs to first be created outside this function
            (trans, rot) = listener.lookupTransform(ar_tags[-1], ar_tags[i], rospy.Time(0))
            trans = np.array(trans)
            if i not in trans_old:
                trans_old[i] = trans
                trans_curr[i] = trans
            else:
                trans_curr[i] = 0.9*trans_old[i] + 0.1*trans
                trans_old[i] = trans

            j += 1
        except Exception as e:
            #print e
            if i in trans_old:
                j += 1
            continue


def pub_vel(zumy_vel, vx = 0.0, vy = 0.0, vz = 0.0, wx = 0.0, wy = 0.0, wz = 0.0):
    # e: twist 6x1 vector
    pub_mssge = Twist()
    pub_mssge.linear.x = vx
    pub_mssge.linear.y = vy
    pub_mssge.linear.z = vz
    pub_mssge.angular.x = wx
    pub_mssge.angular.y = wy  
    pub_mssge.angular.z = wz #right hand rotation              
    zumy_vel.publish(pub_mssge)      

def read_ar_tags(sys):
    ar_tags = {}
    zumy_name = sys.argv[1]
    ar_tags[-1] = 'ar_marker_' + sys.argv[2] # is for Zumy
    Nar = len(sys.argv) - 3
    for i in range(Nar):
        ar_tags[i] = 'ar_marker_' + sys.argv[i+3]    
    print ar_tags
    return (zumy_name, ar_tags, Nar)

def dist(T):
    return math.sqrt((T[0]**2 + T[1]**2))

def d_to_v(dist):
    d = dv_dict()
    return d[dist]


def dv_dict():
    d = {}
    d[1] = 1.04
    d[2] = 1.05
    d[3] = 1.27
    d[4] = 1.75
    d[5] = 1.73
    d[6] = 2.16
    d[7] = 2.65
    d[8] = 2.71
    d[9] = 2.61
    d[10] = 2.41
    d[11] = 2.27
    d[12] = 2.08
    d[13] = 1.94
    d[14] = 1.84
    d[15] = 1.75
    d[16] = 1.63
    d[17] = 1.56
    d[18] = 1.48
    d[19] = 1.42
    d[20] = 1.35
    d[21] = 1.28
    d[22] = 1.24
    d[23] = 1.18
    d[24] = 1.13
    d[25] = 1.11
    d[26] = 1.07
    d[27] = 1.04
    d[28] = 1.02
    d[29] = 0.99
    d[30] = 0.95
    return d

if __name__=='__main__':
    pass
