#!/usr/bin/python
# -*- coding: utf-8 -*-
from E160_state import *
from E160_robot import *
import math
import time



class P_controller:

    def __init__(self, robot, logging = True):
        self.robot = robot  # do not delete this line
        #k constants - for 3 iterations
        #        KP  KB  KA
        #    1  2   -10   10 - Does not meet Strong Stability -
        #    2  2   -10   18 - Meets Strong Stability
        #    3  .5  -8    14 - Does not meet Strong Stability
        #    4  5   -.5   14 - Meets Strong Stability
        #    5  1   -5    5 - Meets Strong Stability

        self.kp = 1  # k_ho
        self.kb = -5  # k_beta
        self.ka = 5 # k_alpha


        self.previous_ctheta = 0
        self.counter = 0
        self.logging = logging

        if(logging == True):
            #all the values
            self.robot.make_headers(['TurnsLeft','kp','kb','ka','dpos_X','dpos_Y','dpos_theta', 'cpos_X','cpos_Y','cpos_theta','Deltax', 'deltay','deltatheta', 'rho', 'alpha', 'beta', 'vix','viy','wi','vr','wr','phi_l','phi_r'])

        self.set_goal_points()

    #Edit goal point list below, if you click a point using mouse, the psoints programmed
    #will be washed outs
    def set_goal_points(self):
        # here is the example of destination code      

       self.robot.state_des.add_destination(x=00,y=200,theta=math.pi/8)    #goal point 1
       self.robot.state_des.add_destination(x=00,y=-200,theta=-math.pi/4) #goal point 2
       self.robot.state_des.add_destination(x=-100,y=0,theta=math.pi/8) #goal point 3
       self.robot.state_des.add_destination(x=-250,y=200,theta=math.pi/4) #goal point 4
       self.robot.state_des.add_destination(x=0,y=200,theta=0.5)    #goal point 5 - back to begginning


    def track_point(self):
        # All d_ means destination
        (d_posX, d_posY, d_theta) = self.robot.state_des.get_des_state()  # get next destination configuration

        # All c_ means current_
        (c_posX, c_posY, c_theta) = self.robot.state.get_pos_state()  # get current position configuration
        difference = c_theta-self.previous_ctheta
        self.previous_ctheta = c_theta

        (c_vix, c_viy, c_wi) = self.robot.state.get_global_vel_state() #get current velocity configuration, in the global frame
        (c_v, c_w) = self.robot.state.get_local_vel_state() #get current local velocity configuration



        # Most of your program should be here, compute rho, alpha and beta using d_pos and c_pos
        deltaX = (d_posX-c_posX)
        deltaY = (d_posY-c_posY)
        lamda = math.atan2(deltaY, deltaX)
        theta = (c_theta+d_theta)
        rho = math.sqrt(math.pow(deltaX,2)+math.pow(deltaY,2))

        alpha = subAngle(lamda,theta)
        #alpha = normalizeAngle(alpha)  # %normalizeAngle   set angle to the range [-pi,pi)

        print("og alpha: ", alpha)

        if (alpha <= math.pi/2 or alpha >= -math.pi/2 ): # go forward
            alpha = normalizeAngle(alpha)
            beta = -theta - alpha
            beta = normalizeAngle(beta)
                #%normalizeAngle   set angle to the range [-pi,pi)

            #calcualte wheelspeed
            c_v = self.kp*rho
            c_w = self.ka*alpha + self.kb*beta

        else: #go backward
            alpha = subangle(math.atan2(-deltay, -deltax),theta) -math.pi
            alpha= normalizeAngle(alpha) #%normalizeangle   set angle to the range [-pi,pi)


            beta = -theta- alpha
            beta= normalizeAngle(beta)    #%normalizeAngle   set angle to the range [-pi,pi)


            #calcualte wheelspeed
            c_v = -self.kp*rho
            c_w = self.ka*alpha + self.kb*beta


        turnsLeft = str(self.robot.state_des.get_des_size() - self.robot.state_des.p)
        print("turns left#: "+ turnsLeft)
        print("c_theta: ", c_theta)
        print("d_theta: ", d_theta)
        print("theta: ", theta)
        print("deltaX: ", deltaX)
        print("deltaY: ", deltaY)
        print("new alpha: ", alpha)
        print("beta: ", beta)

        # self.robot.set_motor_control(linear velocity (cm), angular velocity (rad))
        self.robot.set_motor_control(c_v, c_w)  # use this command to set robot's speed in local frame

        #Calculate wheel rotation speed
        r= 3
        L = 24
        phi_r = c_v / r + (c_w*L)/r
        phi_l = (c_v - c_w*L)/r

        self.robot.send_wheel_speed(phi_l,phi_r ) #unit rad/s
        # use the following to log the variables, use [] to bracket all variables you want to store

        if difference < 0.0001:
            self.counter+=1
        # stored values are in log folder
        if self.logging == True:
            self.robot.log_data([turnsLeft, self.kp,self.kb,self.ka,d_posX,d_posY,d_theta, c_posX,c_posY,c_theta, deltaX, deltaY,abs(d_theta - c_theta),  rho,  alpha, beta, c_vix,c_viy,c_wi,c_v,c_w,phi_l,phi_r])

        threshold = 3 # threshold constant for directoin
        radsthreshold = 0.14 # threshold constant for oreitnation (0.14 rads = 8 degrees)
        if (abs(rho) <threshold and abs(c_theta - d_theta)<radsthreshold) or(self.counter > 200  and  abs(rho) <threshold): #you need to modify the reach way point criteria
            if(self.robot.state_des.reach_destination()):
                print("final goal reached")
                self.robot.set_motor_control(.0, .0)  # stop the motor
                self.counter = 0
                return True
            else:
                print("one goal point reached, continute to next goal point")
                self.counter = 0

        return False

def normalizeAngle(angle):
    if(angle>math.pi):   #%normalizeAngle   set angle to the range [-pi,pi)
        angle -= math.pi*2
    elif(angle<-math.pi):
        angle += math.pi*2

    return angle

def normalizeAngle2(angle):
    if(angle>math.pi/2):   #%normalizeAngle   set angle to the range [-pi,pi)
        angle -= math.pi
    elif(angle<-math.pi/2):
        angle += math.pi
    return angle

def subAngle(angle1,angle2):
    a = angle1 - angle2
    a = normalizeAngle(a)
    return a



