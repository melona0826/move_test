#!/usr/bin/env python
import rospy
from std_srvs.srv import SetBool

def go(trig) :
    rospy.wait_for_service('move_next_position')
    print("connect !")
    move = rospy.ServiceProxy('move_next_position' , SetBool )
    resp = move(trig)
    print("moving !")

    return resp.success


if __name__ == "__main__" :
    print("Inital Pose ...")
    go(True)
    while(True) :
        trg = raw_input("Next ? y/n : ")[0]
        if trg == 'y' :
            x = True
            print("Strat")
            print(go(x))
        else :
            print("Fall !")
