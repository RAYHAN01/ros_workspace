from self_parking_lib import *
class self_parking:
    def run(self):
        self.listener = tf.TransformListener()
        zumy_vel = rospy.Publisher('%s/cmd_vel' % self.zumy, Twist, queue_size=2)
        rate = rospy.Rate(10)
        print "RUNNING PERPENDICULAR PARKING MULTI"
       
        error_old = None
        trans_old = {}
        trans_curr = {} #each key is the translation information (type np.array) of a tag
        Nar = len(self.ar_tags) - 1 # not include zumy tag

        Ntrans = 3
        #print self.ar_tags

        parking_available = False

        (LI,LO,RI,RO) = (Nar - 4, Nar - 3,Nar - 2,Nar - 1)
        (y,x) = (1,0)
        # print "LOOKING FOR AN EMPTY LOT"
        while not parking_available:
            ar_tags_sel = (LI,LO,RI,RO)
            print "FACING TO A LOT"
            while not rospy.is_shutdown():
                cal_trans_curr(self.listener, self.ar_tags, ar_tags_sel, trans_old, trans_curr)
                M1 = (trans_curr[RI] + trans_curr[LI])/2
                M2 = (trans_curr[RO] + trans_curr[LO])/2
                T = M2+0.7*(M2 - M1)
                if abs(T[y]) > 0.01 or T[x] < 0: 
                    pub_vel(zumy_vel, wz = 0.2*T[y]/abs(T[y]))                                                    
                else:
                    break
                rate.sleep()
            pub_vel(zumy_vel, wz = 0) 

            print "MOVING TO A LOT"
            while not rospy.is_shutdown():
                cal_trans_curr(self.listener, self.ar_tags, ar_tags_sel, trans_old, trans_curr)
                M1 = (trans_curr[RI] + trans_curr[LI])/2
                M2 = (trans_curr[RO] + trans_curr[LO])/2
                T = M2+0.7*(M2 - M1)
                if abs(T[x]) > 0.01: 
                    if abs(T[y]) > 0.01:
                        wz = 0.2*T[y]/abs(T[y])
                    else:
                        wz = 0
                    pub_vel(zumy_vel, vx = 0.02, wz = wz)            
                else:
                    break
                rate.sleep()
            pub_vel(zumy_vel, wz = 0) 

            print "FACING RIGHT SENSOR TO THE LOT"
            maxTx = None
            while not rospy.is_shutdown():
                cal_trans_curr(self.listener, self.ar_tags, ar_tags_sel, trans_old, trans_curr)
                T = (trans_curr[RI] + trans_curr[LI])/2
                if T[y] > 0 or abs(T[x]) > 0.01: 
                    pub_vel(zumy_vel, wz = 0.2*T[x]/abs(T[x]))                                                    
                else:
                    pub_vel(zumy_vel, wz = 0)
                    break
                rate.sleep()            
            pub_vel(zumy_vel, wz = 0)

            print "CHECKING THE AVAILABILITY"
            volt_d = 0.0 
            init_time = time.time()            
            while not rospy.is_shutdown():
                if not volt_d:
                    volt_d += self.irR_volt
                else:
                    volt_d = 0.75*volt_d + 0.25*self.irR_volt
                if time.time() - init_time > 2:
                    break
            print volt_d
            if volt_d < 0.22:
                print "THIS LOT IS AVAILABLE"
                parking_available = True
                break
            else:
                if LI - 2 < 0:
                    break
                else:
                    LI -= 2
                    LO -= 2
                    RO -= 2
                    RI -= 2                   
                    continue
        if not parking_available:
            print "NO PARKING LOT IS AVAILABLE"
        else:
            ar_tags_sel = (LI,LO,RI,RO)
            print "BACK FACING TO THE ENTRANCE"
            while not rospy.is_shutdown():
                cal_trans_curr(self.listener, self.ar_tags, ar_tags_sel, trans_old, trans_curr)
                # T = (trans_curr[RI] + trans_curr[LI])/2
                T = trans_curr[RI] + 1.2*(trans_curr[RI] - trans_curr[LI])

                if T[y] < 0 or T[x] > 0: 
                    pub_vel(zumy_vel, wz = -0.2*T[y]/abs(T[y]))                                                    
                else:
                    break
                rate.sleep()                
            pub_vel(zumy_vel, wz = 0) 

            print "BACKING IN PARKING LOT"
            while not rospy.is_shutdown():        
                cal_trans_curr(self.listener, self.ar_tags, ar_tags_sel, trans_old, trans_curr)
                T = (trans_curr[RI] + trans_curr[LI])/2

                #print 'd is' + str(d)
                if dist(T) >0.08: 
                    if abs(T[y]) > 0.01:
                        wz = -0.1*T[y]/abs(T[y])
                    else:
                        wz = 0                   
                    pub_vel(zumy_vel, vx = -0.05,wz = wz)             
                else:
                    print "Done"
                    break
                rate.sleep()  

        # Stop the zumy
        pub_vel(zumy_vel, wz = 0)   

    def __init__(self, zumy, ar_tags):
        rospy.init_node('perpendicular_parking_multi')
        self.zumy = zumy
        self.ar_tags = ar_tags
        self.listener = None 
        #rospy.Subscriber('ai16', Float32, self.irF_callback,queue_size=1)
        #rospy.Subscriber('ai17', Float32, self.irB_callback,queue_size=1)
        #rospy.Subscriber('ai18', Float32, self.irL_callback,queue_size=1)
        rospy.Subscriber('/' + self.zumy + '/ai19', Float32, self.irR_callback,queue_size=1)

    def irB_callback(self, messages):
        self.irB_volt = messages.data
    def irR_callback(self, messages):
        self.irR_volt = messages.data
    def irF_callback(self, messages):
        self.irF_volt = messages.data
    def irL_callback(self, messages):
        self.irL_volt = messages.data


if __name__=='__main__':
    if len(sys.argv) != 11 and len(sys.argv) != 7:
        print('Use: perpendicular_parking_multi.py [ zumy name ] [ AR tag number for Zumy] [Other AR tags number]')
        sys.exit()
    else:
        (zumy_name,ar_tags,Nar) = read_ar_tags(sys)     
        node = self_parking(zumy=zumy_name, ar_tags=ar_tags)
        node.run()
    #input order of artags (four) for one parkinglot: leftup, leftdown, rightup, rightdown
    #input order of artags (six) for two parkinglots: leftup, leftdown, middledown, middleup, rightup, rightdown
    #input order of artags (eight) for three parkinglots: leftup, leftdown, leftmiddledown, leftmiddleup, rightmiddleup, rightmiddledown, rightdown, rightup



