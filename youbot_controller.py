"""youbot_controller controller."""

from controller import Robot, Motor, Camera, Accelerometer, GPS, Gyro, LightSensor, Receiver, RangeFinder, Lidar
from controller import Supervisor

from youbot_zombie import *
import numpy as np

#constants from objects array
#change based on output from objects
BERRY = "BERRY" 
ZOMBIE = "ZOMBIE"
HEALTH_BERRY = None
ENERGY_BERRY = None
# Keep track of berry effects with global dictionary
# Format is ("primary_effect", "second_effect") 
berry_effects = {}
berry_effects["RED_BERRY"] = ("", "")
berry_effects["YELLOW_BERRY"] = ("", "")
berry_effects["ORANGE_BERRY"] = ("", "")
berry_effects["PINK_BERRY"] = ("", "")
#------------------CHANGE CODE BELOW HERE ONLY--------------------------
#define functions here for making decisions and using sensor inputs

#randomly wanders when no objects detected
#takes the wheel objects as parameters


#records all objects detected in camera fov or lidar and their attributes: {distance, compass direction, type, subtype}
objects = []

#collects vectors from behaviors
#MIGHT MOVE THIS TO INSISDE ROBOT LOOP
vectors = []

# helper functions for extracting distance and heading given lidar information
def compute_heading(x, z): return np.arctan(z / x)
def compute_euclidian_distance(x, z): return np.sqrt(x ** 2 + z ** 2)

class Vector:
    def __init__(self, distance, heading):
        self.distance = distance
        self.heading = heading

class Object:
    def __init__(self, distance, heading, risk_factor=0, attraction_factor=1):
        # placement of object relative to robot
        self.location_vector = Vector(distance, heading)
        self.update_risk(risk_factor, attraction_factor)
      
    def update_risk(self, risk_factor):
        # risk factor assesses danger based on distance from object
        self.risk_factor = risk_factor
        # update risk_vecotr with new risk factor and attraction_factor = -1 or 1
        self.risk_vector = Vector(distance * risk_factor, heading * self.attraction_factor)
        
class Zombie(Object):
    def __init__(self, distance, heading)
        self.object_class = "ZOMBIE"
        self.risk_factor = 2
        self.attraction_factor = -1
        super().__init__(distance, heading, risk_factor = self.risk_factor)

class Berry(Object):
    def __init__(self, distance, heading):
        self.object_class = "BERRY"
        self.risk_factor = 0
        self.attraction_factor = 1
        super().__init__(distance, heading, risk_factor = self.risk_factor)

class RedBerry(Berry):
  def __init__(self, distance, heading):
    self.object_class_subtype = "RED_BERRY"
    self.primary_effect = ""
    self.secondary_effect = ""
    super().__init__(distance, heading, risk_factor = self.risk_factor)

class YellowBerry(Berry):
  def __init__(self, distance, heading):
    self.object_class_subtype = "YELLOW_BERRY"
    self.primary_effect = ""
    self.secondary_effect = ""
    super().__init__(distance, heading, risk_factor = self.risk_factor)

class OrangeBerry(Berry):
  def __init__(self, distance, heading):
    self.object_class_subtype = "ORANGE_BERRY"
    self.primary_effect = ""
    self.secondary_effect = ""
    super().__init__(distance, heading, risk_factor = self.risk_factor)

class PinkBerry(Berry):
  def __init__(self, distance, heading):
    self.object_class_subtype = "PINK_BERRY"
    self.primary_effect = ""
    self.secondary_effect = ""
    super().__init__(distance, heading, risk_factor = self.risk_factor)

class AquaZombie(Zombie):
    def __init__(self, distance, heading):
      self.object_class_subtype = "AQUA_ZOMBIE"
      super().__init__(distance, heading, risk_factor = self.risk_factor)

class PurpleZombie(Zombie):
    def __init__(self, distance, heading):
      self.object_class_subtype = "PURPLE_ZOMBIE"
      super().__init__(distance, heading, risk_factor = self.risk_factor)

class GreenZombie(Zombie):
    def __init__(self, distance, heading):
      self.object_class_subtype = "GREEN_ZOMBIE"
      super().__init__(distance, heading, risk_factor = self.risk_factor)

class BlueZombie(Zombie):
    def __init__(self, distance, heading):
      self.object_class_subtype = "BLUE_ZOMBIE"
      super().__init__(distance, heading, risk_factor = self.risk_factor)
        
#--helper--
#Returns closest object of given type
def checkForItem(objects, typeOfObject, *args):
  distance = 11
  itemFound = None
  for item in objects:
    if len(args) == 0:
      if item.object_class == typeOfObject and item.distance < distance:
        itemFound = item
    else:
      if item.object_class_subtype == args[0] and item.distance < distance:
        itemFound = item
        
  return itemFound

#--Behavior--
#wander when no objects in sight
def wander(objects):
  heading, magnitude = 0, 0
  
  if len(objects) == 0:
    heading = np.random.normal(1, 0.25) * 3.141596
    magnitude = 0.2
  
  return heading, magnitude

#--Behavior--
#pursues the berry that is needed the most, or the closest berry
def pursueBerry(objects, robot_info):
  #store target berry object
  berry = None 
  distance = 11
  heading, magnitude = 0, 0
  #UPDATE HOW YOU USE OBJECT LiST 
  #DETERMINE HOW TO LEARN WHAT EACH BERRY DOES

  for item in objects:
    robot_health = robot_info[0]
    robot_energy = robot_info[1]

    # only consume a berry if you need it: health < 80 and energy < 60
    if item.object_class == "BERRY" and robot_health < 80 and robot_energy < 60:
      berry_primary_effect = berry_effects[item.object_class_subtype][0]
      berry_secondary_effect = berry_effects[item.object_class_subtype][1]
      
      #--Low health cases--
      if robot_health < 60:
        # if health is low and berry detected is known to give health, prioritize
        if berry_primary_effect == "+20 HEALTH":
          berry = item
      elif robot_health < 30:
        #if health is VERY low and berry detected has possible side effect of extra health, prioritize
        if berry_primary_effect == "+20 HEALTH" or berry_secondary_effect == "+20 HEALTH":
          berry = item
      
      #--Low energy cases--
      if robot_energy < 50:
         # if low energy and berry detected is known to give energy, prioritize
        if berry_primary_effect == "+40 ENERGY":
          berry = item
      elif robot_energy < 20:
        #if energy is VERY low and berry detected has possible side effect of extra energy, prioritize
        if berry_primary_effect == "+40 ENERGY" or berry_secondary_effect == "+40 ENERGY":
          berry = item
        
      #--Closest berry case (Only do if robot cannot find another berry) --
      if item.distance < distance and not berry:
        berry = item
        distance = item.distance
    
  if berry is not null:
    objects.remove(berry)
    berry.item.update_risk(0.5 * berry.distance)
    objects.append(berry)

#--Behavior--
def noise():
  magnitude = 0.01
  heading = np.random.normal(0, 0.25)
  return heading, magnitude
   

#--Behavior--
#turns towards a detected solid (not zombie) in the lidar to classify it
def goToSolid(objects):
  target = None
  distance = 11
  #if there's no berries detected
  if  checkForItem(objects, BERRY) is None:
    for item in objects:
      if item.object_class != ZOMBIE:
        if item.distance < distance #get closest object
          target = item
          distance = item.distance

  if target is not None:
    objects.remove(target)
    target.attraction_factor = 0.1 * target.distance
    objects.append(target)

#--Behavior--
def avoidZombie(objects, robot_info):
  #go in direction that is the sum of the opposite vectors of all zombies in camera fov

  # If low health, the risk factor for each zombie drastically increases
  risk_multiplier = 1
  if robot_info[0] < 50:
    risk_multiplier = 2
  elif robot_info[0] < 30:
    risk_multiplier = 3

  for item in objects:
    if item.object_class == "ZOMBIE":
      if item.object_class_subtype == "GREEN_ZOMBIE":
        # green zombies chase if you are near and move randomly when far
        item.update_risk(0.5 * item.distance * risk_multiplier)
      elif item.object_class_subtype == "BLUE_ZOMBIE":
        # will relentlessly chase robot
        item.update_risk(0.3 * item.distance * risk_multiplier)
      elif item.object_class_subtype == "AQUA_ZOMBIE":
        if item.distance < 4:
          # only generate when in 3m range
          item.update_risk(1 * item.distance * risk_multiplier)
      elif item.object_class_subtype == "PURPLE_ZOMBIE":
        # If purple zombie, call helper func
        avoidPurpleZombie(objects, item, risk_multiplier)
      else:
          # RUN

#--Behavior--
#pursue a berry in the opposite direction of purple zombie
def avoidPurpleZombie(objects, item, risk_multiplier):
  #check if there's a berry
  targetBerry = checkForItem(objects, BERRY)
  while targetBerry is not None and targetBerry.distance > 1:
    #go towards berry 
    
  #if close to berry, turn Right or left

  #if no berry, just have the usual opposite vector 

  
  
  return













#------------------CHANGE CODE ABOVE HERE ONLY--------------------------

def main():
    robot = Supervisor()

    # get the time step of the current world.
    timestep = int(robot.getBasicTimeStep())
    
    #health, energy, armour in that order 
    robot_info = [100,100,0]
    passive_wait(0.1, robot, timestep)
    pc = 0
    timer = 0
    
    robot_node = robot.getFromDef("Youbot")
    trans_field = robot_node.getField("translation")
    
    get_all_berry_pos(robot)
    
    robot_not_dead = 1
    
    #------------------CHANGE CODE BELOW HERE ONLY--------------------------
    
    #COMMENT OUT ALL SENSORS THAT ARE NOT USED. READ SPEC SHEET FOR MORE DETAILS
    #accelerometer = robot.getDevice("accelerometer")
    #accelerometer.enable(timestep)
    
    #gps = robot.getDevice("gps")
    #gps.enable(timestep)
    
    compass = robot.getDevice("compass")
    compass.enable(timestep)
    
    #camera1 = robot.getDevice("ForwardLowResBigFov")
    #camera1.enable(timestep)
    
    #camera2 = robot.getDevice("ForwardHighResSmallFov")
    #camera2.enable(timestep)
    
    #camera3 = robot.getDevice("ForwardHighRes")
    #camera3.enable(timestep)
    
    #camera4 = robot.getDevice("ForwardHighResSmall")
    #camera4.enable(timestep)
    
    #camera5 = robot.getDevice("BackLowRes")
    #camera5.enable(timestep)
    
    #camera6 = robot.getDevice("RightLowRes")
    #camera6.enable(timestep)
    
    #camera7 = robot.getDevice("LeftLowRes")
    #camera7.enable(timestep)
    
    #camera8 = robot.getDevice("BackHighRes")
    #camera8.enable(timestep)
    
    #gyro = robot.getDevice("gyro")
    #gyro.enable(timestep)
    
    #lightSensor = robot.getDevice("light sensor")
    #lightSensor.enable(timestep)
    
    #receiver = robot.getDevice("receiver")
    #receiver.enable(timestep)
    
    #rangeFinder = robot.getDevice("range-finder")
    #rangeFinder.enable(timestep)
    
    lidar = robot.getDevice("lidar")
    lidar.enable(timestep)
    
    fr = robot.getDevice("wheel1")
    fl = robot.getDevice("wheel2")
    br = robot.getDevice("wheel3")
    bl = robot.getDevice("wheel4")
    
    fr.setPosition(float('inf'))
    fl.setPosition(float('inf'))
    br.setPosition(float('inf'))
    bl.setPosition(float('inf'))
    
    
    i=0
           

    #------------------CHANGE CODE ABOVE HERE ONLY--------------------------
    
    
    while(robot_not_dead == 1):
        
        if(robot_info[0] < 0):
           
            robot_not_dead = 0
            print("ROBOT IS OUT OF HEALTH")
            #if(zombieTest):
            #    print("TEST PASSED")
            #else:
            #    print("TEST FAILED")
            #robot.simulationQuit(20)
            #exit()
            
        if(timer%2==0):
            trans = trans_field.getSFVec3f()
            robot_info = check_berry_collision(robot_info, trans[0], trans[2], robot)
            robot_info = check_zombie_collision(robot_info, trans[0], trans[2], robot)
            
        if(timer%16==0):
            robot_info = update_robot(robot_info)
            timer = 0
        
        if(robot.step(timestep)==-1):
            exit()
            
            
        timer += 1
        
     #------------------CHANGE CODE BELOW HERE ONLY--------------------------   
        #called every timestep
        curr_health = robot_info[0]
        curr_energy = robot_info[1]
        curr_armor = robot_info[2]

        # previous berry can only have 1 effect
        if (curr_health - prev_health >= 18):
          # classify last berry as +20 HEALTH
        elif(curr_energy - prev_energy <= -18):
          # classify last berry as -20 ENERGY
        elif(curr_energy - prev_energy >= 38):
          # classify last berry as +40 ENERGY
        elif(curr_armor >= 14):
          # classify last berry as +15 ARMOR

        
        
        #possible pseudocode for moving forward, then doing a 90 degree left turn
        #if i <100
            #base_forwards() -> can implement in Python with Webots C code (/Zombie world/libraries/youbot_control) as an example or make your own
        
        #if == 100 
            # base_reset() 
            # base_turn_left()  
            #it takes about 150 timesteps for the robot to compli ete the turn
                 
        #if i==300
            # i = 0
        
        #i+=1
        prev_health = curr_health
        prev_energy = curr_energy
        #make decisions using inputs if you choose to do so
         
        #------------------CHANGE CODE ABOVE HERE ONLY--------------------------
        
        
    return 0   


main()
