import numpy as np
import math
from scipy.optimize import fsolve
from scipy import integrate
import matplotlib.pyplot as plt
import pandas as pd
import serial
import time
from scipy.integrate import quad
from concurrent.futures import ProcessPoolExecutor
import os

#curve functions.
def p(u): #curve to be printed.
    #the following function constains the curve to be evaluated.
    #the function takes in a time t and returns the position and velocity components for that time.
    #evaluating position.
    x = u
    y = 0
    #evaluating velocity.
    dx = 1
    dy = 0
    #calculating the tangent vector
    ta = np.arctan2(dy, dx)
    #returning the position and velocity components
    return x,y,dx,dy,ta
def b(u): #path for platform to follow.
    #the following function constains the curve to be evaluated.
    #the function takes in a time t and returns the position and velocity components for that time.
    #evaluating position.
    x = u
    y = 0
    #evaluating velocity.
    dx = 1
    dy = 0
    #calculating the tangent vector
    ta = np.arctan2(dy, dx)
    #returning the position and velocity components
    return x,y,dx,dy,ta
def v(u,pah): #curve tangent velocity (meters/second).
    #the following function caculates the tangent velocity of the curve.
    #the function takes in a time t and returns the tangent velocity.
    #obtainging velocity components from the curve.
    dx,dy = pah(u)[2],pah(u)[3]
    #calculating the tangent velocity.
    v = np.sqrt(dx**2+dy**2)
    #returning the tangent velocity.
    return v
def s(u,pah): #curve arclenght (meters).
    #the following function numerically calculates the arclenght of the curve.
    #the function takes in u and returns the arclenght.
    #calculating the arc lenght numerically.
    s = quad(lambda u:v(u,pah),0,u,limit=1000)[0]
    #returning the arc length.
    return s
def u(t,pah,mtv): #calculates the value for u.
    #the following functions solves for the value of u in wich the curve has reached a certain arclength.
    #the function takes in a time t and returns u.
    #solving for u.
    u = fsolve(lambda u:s(u,pah)-mtv*t,[1])[0]
    #returning u.
    return u
def gtl(xg,yg,ang,x0,y0,an0): #global to local transformation.
    xl = (xg-x0)*np.cos(an0) + (yg-y0)*np.sin(an0)
    yl = -(xg-x0)*np.sin(an0) + (yg-y0)*np.cos(an0)
    anl = ang - an0 
    return xl,yl,anl
def ltg(xl,yl,anl,x0,y0,an0): #local to global transformation.
    xg = xl*np.cos(an0) - yl*np.sin(an0)
    yg = xl*np.sin(an0) + yl*np.cos(an0)
    xg = xg + x0
    yg = yg + y0
    ang = anl + an0
    return xg,yg,ang

#constants.
clock_interval = 1/1000000 #set time step to 1 microsecond.
total_print_time = 12 #total print time.
update_frequency = 1000 #update frequency.
point_feed_rate = 50 #point feed rate.
vel = 0.277 #printhead velocity.
plv = 0.277 #platform velocity.
steps_per_rev = 3200
r = -10
kp,kd = r**2,-2*r

def wrapper1(t):
        return p(u(t,p,vel))
def wrapper2(t):
        return b(u(t,b,plv))

if __name__ == '__main__':
    with ProcessPoolExecutor(max_workers=4) as executor:
        gantry_position_array = list(executor.map(wrapper1, np.linspace(0,total_print_time,num=int(total_print_time*point_feed_rate))))  
    gantry_position_array = np.array(gantry_position_array)  
    with ProcessPoolExecutor(max_workers=4) as executor:
        platform_position_array = list(executor.map(wrapper2, np.linspace(0,total_print_time,num=int(total_print_time*point_feed_rate))))
    platform_position_array = np.array(platform_position_array)

    gantry_x = np.zeros(int(total_print_time*point_feed_rate)-1)
    gantry_y = np.zeros(int(total_print_time*point_feed_rate)-1)
    gantry_theta = np.zeros(int(total_print_time*point_feed_rate)-1)
    gantry_x_velocities = np.zeros(int(total_print_time*point_feed_rate)-1)
    gantry_y_velocities = np.zeros(int(total_print_time*point_feed_rate)-1)
    gantry_theta_velocities = np.zeros(int(total_print_time*point_feed_rate)-1)
    gantry_x_accelerations = np.zeros(int(total_print_time*point_feed_rate)-1)
    gantry_y_accelerations = np.zeros(int(total_print_time*point_feed_rate)-1)
    gantry_theta_accelerations = np.zeros(int(total_print_time*point_feed_rate)-1)
    gantry_x_target = np.zeros(int(total_print_time*point_feed_rate)-1)
    gantry_y_target = np.zeros(int(total_print_time*point_feed_rate)-1)
    gantry_theta_target = np.zeros(int(total_print_time*point_feed_rate)-1)
    gantry_x_target_velocities = np.gradient(gantry_position_array[:,0])*point_feed_rate
    gantry_y_target_velocities = np.gradient(gantry_position_array[:,1])*point_feed_rate
    gantry_theta_target_velocities = np.gradient(gantry_position_array[:,4])*point_feed_rate
    gantry_x_target_accelerations = np.gradient(gantry_x_target_velocities)*point_feed_rate
    gantry_y_target_accelerations = np.gradient(gantry_y_target_velocities)*point_feed_rate
    gantry_theta_target_accelerations = np.gradient(gantry_theta_target_velocities)*point_feed_rate
    time_slots = np.zeros(int(total_print_time*point_feed_rate)-1)
    point_indexer = 0

    #setup for main loop
    simulated_time = np.linspace(0,total_print_time,num=int(1/clock_interval*total_print_time))
    prev_micros_update,update_interval = 0,1/update_frequency
    prev_micros_target,target_interval = 0,1/point_feed_rate
    angle1,stepper_position1 = 0,gantry_position_array[0][0]*steps_per_rev
    angle2,stepper_position2 = 0,gantry_position_array[0][1]*steps_per_rev
    angle3,stepper_position3 = 0,gantry_position_array[0][4]*steps_per_rev
    angular_velocity1,angular_acceleration1 = 0.01,0
    angular_velocity2,angular_acceleration2 = 0.01,0
    angular_velocity3,angular_acceleration2 = 0.01,0
    pe1,de1,prev_pe1 = 0,0,0
    pe2,de2,prev_pe2 = 0,0,0
    pe3,de3,prev_pe3 = 0,0,0
    prev_micros_step1,stepper_interval1 = 0,32767
    prev_micros_step2,stepper_interval2 = 0,32767
    prev_micros_step3,stepper_interval3 = 0,32767
    #input("Make any input to start sim")

    #put impact into platform position array
    impactpoint = 250
    for i in range(50):
        platform_position_array[i+impactpoint][0] = platform_position_array[i+impactpoint][0] - (66.26/60) * (50-i)/50

    print("Sim has started")
    for i in simulated_time:
        if i >= prev_micros_target + target_interval:
            gantry_x[point_indexer],gantry_y[point_indexer],gantry_theta[point_indexer] = angle1,angle2,angle3
            gantry_x_velocities[point_indexer],gantry_y_velocities[point_indexer],gantry_theta_velocities[point_indexer] = angular_velocity1,angular_velocity2,angular_velocity3
            gantry_x_accelerations[point_indexer],gantry_y_accelerations[point_indexer],gantry_theta_accelerations[point_indexer] = angular_acceleration1,angular_acceleration2,angular_acceleration3
            time_slots[point_indexer] = i
            point_indexer += 1
            prev_micros_target = i

        if i >= prev_micros_update + update_interval:
            local_position = gtl(gantry_position_array[point_indexer][0],gantry_position_array[point_indexer][1],gantry_position_array[point_indexer][4],platform_position_array[point_indexer][0],platform_position_array[point_indexer][1],platform_position_array[point_indexer][4])
            angle1 = stepper_position1/steps_per_rev
            angle2 = stepper_position2/steps_per_rev
            angle3 = stepper_position3/steps_per_rev
            pe1 = local_position[0] - angle1
            pe2 = local_position[1] - angle2
            pe3 = local_position[2] - angle3
            de1 = (pe1-prev_pe1)/update_interval
            de2 = (pe2-prev_pe2)/update_interval
            de3 = (pe3-prev_pe3)/update_interval
            prev_pe1 = pe1
            prev_pe2 = pe2
            prev_pe3 = pe3
            angular_acceleration1 = pe1*kp+de1*kd + gantry_x_target_accelerations[point_indexer]
            angular_acceleration2 = pe2*kp+de2*kd + gantry_y_target_accelerations[point_indexer]
            angular_acceleration3 = pe3*kp+de3*kd + gantry_theta_target_accelerations[point_indexer]
            angular_velocity1 += angular_acceleration1 * update_interval
            angular_velocity2 += angular_acceleration2 * update_interval
            angular_velocity3 += angular_acceleration3 * update_interval
            angular_velocity1 = np.clip(angular_velocity1,-5,5)
            angular_velocity2 = np.clip(angular_velocity2,-5,5)
            angular_velocity3 = np.clip(angular_velocity3,-5,5)
            stepper_interval1 = 1.0/(abs(angular_velocity1)*steps_per_rev)
            stepper_interval2 = 1.0/(abs(angular_velocity2)*steps_per_rev)
            stepper_interval3 = 1.0/(abs(angular_velocity3)*steps_per_rev)
            prev_micros_update = i
        
        #stepper simulator 1
        if i >= prev_micros_step1 + stepper_interval1:
            if angular_velocity1>0:
                stepper_position1 += 1
            else:
                stepper_position1 -= 1
            prev_micros_step1 = i

        #stepper simulator 2
        if i >= prev_micros_step2 + stepper_interval2:
            if angular_velocity2>0:
                stepper_position2 += 1
            else:
                stepper_position2 -= 1
            prev_micros_step2 = i

        #stepper simulator 3
        if i >= prev_micros_step3 + stepper_interval3:
            if angular_velocity3>0:
                stepper_position3 += 1
            else:
                stepper_position3 -= 1
            prev_micros_step3 = i

    #calculate global positions for gantry.
    gantry_x_global,gantry_y_global,gantry_theta_global = ltg(gantry_x,gantry_y,gantry_theta,platform_position_array[:,0][:-1],platform_position_array[:,1][:-1],platform_position_array[:,4][:-1])

    sim_data = {
        'Time [s]': time_slots,
        #positions
        'Gantry x local [revs]': gantry_x,
        'Gantry y local [revs]': gantry_y,
        'Gantry theta local [revs]': gantry_theta,
        'Gantry x global [revs]': gantry_x_global,
        'Gantry y global [revs]': gantry_y_global,
        'Gantry theta global [revs]': gantry_theta_global,
        'Platform x global [revs]': platform_position_array[:,0][:-1],
        'Platform y global [revs]': platform_position_array[:,1][:-1],
        'Platform theta global [revs]': platform_position_array[:,4][:-1],

        #velocities
        'Gantry x velocity local [revs/s]': gantry_x_velocities,
        'Gantry y velocity local [revs/s]': gantry_y_velocities,
        'Gantry theta velocity local [revs/s]': gantry_theta_velocities,
        'Gantry x velocity global [revs/s]': np.gradient(gantry_x_global)*50,
        'Gantry y velocity global [revs/s]': np.gradient(gantry_y_global)*50,
        'Gantry theta velocity global [revs/s]': np.gradient(gantry_theta_global)*50,
        'Platform x velocity global [revs/s]': np.gradient(platform_position_array[:,0][:-1])*50,
        'Platform y velocity global [revs/s]': np.gradient(platform_position_array[:,1][:-1])*50,
        'Platform theta velocity global [revs/s]': np.gradient(platform_position_array[:,4][:-1])*50,

        #accelerations
        'Gantry x acceleration local [revs/s^2]': gantry_x_accelerations,
        'Gantry y acceleration local [revs/s^2]': gantry_y_accelerations,
        'Gantry theta acceleration local [revs/s^2]': gantry_theta_accelerations,
        'Gantry x acceleration global [revs/s^2]': np.gradient(np.gradient(gantry_x_global)*50)*50,
        'Gantry y acceleration global [revs/s^2]': np.gradient(np.gradient(gantry_y_global)*50)*50,
        'Gantry theta acceleration global [revs/s^2]': np.gradient(np.gradient(gantry_theta_global)*50)*50,
        'Platform x acceleration [revs/s^2]': np.gradient(np.gradient(platform_position_array[:,0][:-1])*50)*50,
        'Platform y acceleration [revs/s^2]': np.gradient(np.gradient(platform_position_array[:,1][:-1])*50)*50,
        'Platform theta acceleration [revs/s^2]': np.gradient(np.gradient(platform_position_array[:,4][:-1])*50)*50
    }

    #save simulaton data to excel file
    sim_df = pd.DataFrame(sim_data)
    save_attempts = 0
    while True:
        try:
            sim_file_path = f"C:\\Users\\chris\\OneDrive - Danmarks Tekniske Universitet\\Diplomingeniørprojekt Forår 2025 - General\\Simulation results\\sim_output_{save_attempts}.xlsx"
            sim_df.to_excel(sim_file_path, index=False, engine='openpyxl')
            print(f"Saved sim as: sim_output_{save_attempts}")
            break
        except (PermissionError, OSError) as e:
            save_attempts += 1

    #open a serial line to controller
    while True:
        try:
            ser = serial.Serial('COM4', 115200, timeout=1)
            time.sleep(1)
            print("Succesfully connected to controller")
            break
        except:
            print("Unable to connect to controller")
            time.sleep(1)

    #send regulator constants to microcontroller.
    values = (kp, 0, kd)
    message = "{:.6f},{:.6f},{:.6f}\n".format(*values)
    ser.write("get_reg_const\n".encode())
    ser.write(message.encode())

    #setup arrays for controller
    controller_axis_1 = np.zeros(int(total_print_time*point_feed_rate)-1)
    controller_axis_2 = np.zeros(int(total_print_time*point_feed_rate)-1)
    controller_axis_3 = np.zeros(int(total_print_time*point_feed_rate)-1)
    #send path to controller
    for i in range(gantry_position_array.shape[0]):
        ax1, ax2, ax3 = platform_position_array[i,0],gantry_position_array[i, 1], gantry_position_array[i, 0]
        message = f"{ax1:.6f},{ax2:.6f},{ax3:.6f}\n"
        ser.write(b"get_data\n")
        ser.write(message.encode('utf-8'))
    #command controller to start running
    input("Make any input to start test")
    ser.write("start_running\n".encode())
    print("Test has started")
    #wait for controller to respond
    line = "0"
    while line != "process_finished":
        line = ser.readline().decode('utf-8').strip()
    #command controller to return data
    ser.write("return_data\n".encode())
    #read data from controller.
    for i in range(gantry_position_array.shape[0]-1):
        controller_axis_1[i] = float(ser.readline().decode('utf-8').strip())
        controller_axis_2[i] = float(ser.readline().decode('utf-8').strip())
        controller_axis_3[i] = float(ser.readline().decode('utf-8').strip())
    test_data = {
    'Time [s]': time_slots,
    #positions
    'Axis 1 [revs]': controller_axis_1,
    'Axis 2 [revs]': controller_axis_2,
    'Axis 3 [revs]': controller_axis_3,
    #velocities
    'Axis 1 velocity [revs/s]': np.gradient(controller_axis_1)*50,
    'Axis 2 velocity [revs/s]': np.gradient(controller_axis_2)*50,
    'Axis 3 velocity [revs/s]': np.gradient(controller_axis_3)*50,
    #accelerations
    'Axis 1 acceleration [revs/s^2]': np.gradient(np.gradient(controller_axis_1)*50)*50,
    'Axis 2 acceleration [revs/s^2]': np.gradient(np.gradient(controller_axis_2)*50)*50,
    'Axis 3 acceleration [revs/s^2]': np.gradient(np.gradient(controller_axis_3)*50)*50
    }

    #save test data to excel file
    test_df = pd.DataFrame(test_data)
    save_attempts = 0
    while True:
        try:
            test_file_path = f"C:\\Users\\chris\\OneDrive - Danmarks Tekniske Universitet\General\\Simulation results\\test_output_{save_attempts}.xlsx"
            test_df.to_excel(test_file_path, index=False, engine='openpyxl')
            print(f"Saved test as: test_output_{save_attempts}")
            break
        except (PermissionError, OSError) as e:
            save_attempts += 1