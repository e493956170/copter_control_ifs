import setup_path 
import airsim

import time

# connect to the AirSim simulator
client = airsim.MultirotorClient()
client.confirmConnection()
client.reset()
client.enableApiControl(True)
client.armDisarm(True)

print("Taking off")
client.moveByVelocityAsync(0, 0, -5,3).join
time.sleep(3)    



   
    
