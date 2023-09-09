"""
ros node that subscribes to 
- a topic for the rover velocity
- a topic for the arm positions to reach
- a topic for the drilling velocities
- jetson power informations from jtop

and generates the energy consumed for each subsystem based on the expected power consumption of the motors

"""

import rospy
import numpy as np
import time
import os
from jtop import jtop
import matplotlib
import matplotlib.pyplot as plt


def read_sensor_data():
    #read energy consumed from sensor
    #covnert to Watt
    return 0


def rover_callback(msg, params):
    v=msg.linear.x
    w=msg.angular.z
    is_still=(v==0 and w==0)
    if not is_still:
        params["in_use"] = "rover"
    else:
        params["in_use"] = None

def arm_callback(msg, params):
    params["in_use"] = "arm"

def drill_callback(msg, params):
    params["in_use"] = "drill"





def main():

    energies={
        "rover":[],
        "arm":[],
        "drill":[],
        "jetson":[],
        "total":[],
        "times":[]
    }

    params={
        "rover_energy_consumed":0, #in Watt
        "arm_energy_consumed":0,
        "drill_energy_consumed":0,
        "jetson_energy_consumed":0,
        "total_energy_consumed":0,

        "rover_topic": "/cmd_vel",
        "in_use": None, #current subsystem in use, one of "rover", "arm", "drill"
    }


    #where to save images
    img_path=os.path.dirname(os.path.dirname(__file__))
    img_path=os.path.join(img_path, "images")


    #rospy.init_node("energy_report_manager", anonymous=True)
    #rospy.subscriber(params["rover_topic"], Twist, lambda msg: rover_callback(msg, params))
    rospy.init_node("energy_report_manager", anonymous=True)
    #pub=rospy.publisher("energy_report", Float32, queue_size=10)
    jetson=jtop()
    jetson.start()
    rate=rospy.Rate(10)
    total_energy=0
    start_time=time.time()
    while True:
        try:
            # jetson.ok() will provide the proper update frequency
            if jetson.ok():
                
                jetson_energy_consumed=jetson.stats['Power TOT'] # in W
                total_energy+=jetson_energy_consumed
                energies["jetson"].append(jetson_energy_consumed)
                # Read tegra stats
                print(jetson_energy_consumed)

                energy_from_sensor=read_sensor_data()  
                total_energy+=energy_from_sensor
                energies[params["in_use"]].append(energy_from_sensor)
                
            energies["total"].append(total_energy)
            energies["times"].append(time.time()-start_time)
            pub.publish(total_energy)
            rate.sleep()
        except KeyboardInterrupt:
            jetson.close()
            break
    
    #using matplotlib generate a plot for each array in nergies dict
    for key in energies.keys():
        plt.plot(energies["times"], energies[key])
        #add axis labels
        plt.xlabel("Time (s)")
        plt.ylabel("Energy (W)")
        plt.title(key + " energy consumption")
        #save to fig
        plt.savefig(os.path.join(os.path.dirname(__file__), "energy_report_"+key+".png"))
        plt.clf()
    #save to csv

    


if __name__ == '__main__':
    main()