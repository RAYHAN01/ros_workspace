from self_parking_lib import *
class self_parking:
    def run(self):
        self.init_params()
        self.listener = tf.TransformListener()
        zumy_vel = rospy.Publisher('%s/cmd_vel' % self.zumy, Twist, queue_size=2)
        rate = rospy.Rate(10)
        print "RUNNING PARALLEL PARKING"
       
        error_old = None
        trans_old = {}
        trans_curr = {}
        Nar = len(self.ar_tags) - 1 # not include zumy tag
        Ntrans = 3
        #print self.ar_tags

        (LI,LO,RI,RO) = (0,1,2,3)
        (y,x) = (1,0)
        ar_tags_sel = (LI,LO,RI,RO)
        print "FACING TO LEFT BOUNDARY"
        while not rospy.is_shutdown():
            cal_trans_curr(self.listener, self.ar_tags, ar_tags_sel, trans_old, trans_curr)
            # print trans_curr    
            T = trans_curr[LO]+ (trans_curr[LO]-trans_curr[LI])

            if abs(T[y]) > 0.1 or T[x] < 0: 
                wz = 0.2*T[y]/abs(T[y])
                pub_vel(zumy_vel, wz = wz,vx = max(0.02,self.minv(wz)))             
            else:
                break
            rate.sleep()
        pub_vel(zumy_vel, wz = 0)

        print "MOVING TO LEFT BOUNDARY"
        while not rospy.is_shutdown():        
            cal_trans_curr(self.listener, self.ar_tags, ar_tags_sel, trans_old, trans_curr)

            T = trans_curr[LO]+ 1.2*(trans_curr[LO]-trans_curr[LI])+0.5*(trans_curr[LI]-trans_curr[RI])

            if abs(T[x]) > 0.01: 
                if abs(T[y]) > 0.01:
                    wz = 0.2*T[y]/abs(T[y])
                else:
                    wz = 0                      
                pub_vel(zumy_vel, vx =max(0.05,self.minv(wz)), wz = wz)            
            else:
                break
            rate.sleep()
        pub_vel(zumy_vel, wz = 0)

        print "BACK FACING TO CORNER"
        while not rospy.is_shutdown():        
            cal_trans_curr(self.listener, self.ar_tags, ar_tags_sel, trans_old, trans_curr)
            T = trans_curr[RI]
            if T[y] < 0  or T[x] > 0: 
                wz = -0.3*T[y]/abs(T[y])
                pub_vel(zumy_vel, wz = wz, vx = -self.minv(wz))             
            else:
                break     
            rate.sleep()   
        pub_vel(zumy_vel, wz = 0)
            
        print "INITIAL BACKING IN PARKING LOT"
        while not rospy.is_shutdown():        
            cal_trans_curr(self.listener, self.ar_tags, ar_tags_sel, trans_old, trans_curr)
            T2 = trans_curr[LI]
            T = trans_curr[RI] 
            #print 'd is' + str(d)
            if dist(T) >0.1:
                if T2[y]<-0.05:
                    wz = 0.3*T2[y]/abs(T2[y])
                else:
                    wz = 0                   
                pub_vel(zumy_vel, vx = -max(0.01,self.minv(wz)),wz = wz)             
            else:
                break
            rate.sleep()  
        pub_vel(zumy_vel, wz = 0)


        print "FINAL FORWARD TO RIGHT ORIENTATION"
        while not rospy.is_shutdown():        
            cal_trans_curr(self.listener, self.ar_tags, ar_tags_sel, trans_old, trans_curr)
 
            T = trans_curr[LI]+(trans_curr[LO] - trans_curr[LI])*0.5
            if dist(T) > 0.12:
                if abs(T[y]) > 0:
                    wz = 0.2*T[y]/abs(T[y])
                else:
                    wz = 0                  
                pub_vel(zumy_vel, vx = max(0.01,self.minv(wz)), wz = wz)     
            else:                                       
                break

            rate.sleep()

        pub_vel(zumy_vel, wz = 0)
    def __init__(self, zumy, ar_tags):
        rospy.init_node('parallel_parking')
        self.zumy = zumy
        self.ar_tags = ar_tags
        self.listener = None 

    def init_params(self):
        e = 0.135
        self.R = e/math.sin(30.0/180.0*math.pi)
        w = 0.15
        Ri = math.sqrt(self.R**2-e**2) - w/2
        pb = 0.065
        pf = 0.2 - e - pb
        Re = math.sqrt((Ri+w)**2+(e+pf)**2)


    def minv(self, w):
        # return abs(w*self.R/(2*math.pi))
        return abs(w*self.R)

    def maxw(self, v):
        # return abs(v*2*math.pi/self.R)
        return abs(v/self.R)

if __name__=='__main__':
    if len(sys.argv) != 7:
        print('Use: parallel_parking.py [ zumy name ] [ AR tag number for Zumy] [Other AR tags number]')
        sys.exit()
    else:
        (zumy_name,ar_tags,Nar) = read_ar_tags(sys)     
        node = self_parking(zumy=zumy_name, ar_tags=ar_tags)
        node.run()
    #input order of artags (four) for one parkinglot: leftup, leftdown, rightup, rightdown
    #input order of artags (six) for two parkinglots: leftup, leftdown, middledown, middleup, rightup, rightdown
    #input order of artags (eight) for three parkinglots: leftup, leftdown, leftmiddledown, leftmiddleup, rightmiddleup, rightmiddledown, rightdown, rightup
