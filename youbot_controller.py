"""youbot_controller controller."""

from controller import Robot, Motor, Camera, Accelerometer, GPS, Gyro, LightSensor, Receiver, RangeFinder, Lidar
from controller import Supervisor

from youbot_zombie import *
import numpy as np
import cv2
import struct

#constants from objects array
#change based on output from objects
BERRY = "BERRY" 
ZOMBIE = "ZOMBIE"
HEALTH_BERRY = None
ENERGY_BERRY = None
# Keep track of berry effects with global dictionary
# Format is "COLOR_BERRY" : [("primary_effect", primary_count), ("second_effect", secondary_count)]

berry_effects = {}
berry_effects["RED_BERRY"] = [["", 0], ["", 0]]
berry_effects["YELLOW_BERRY"] = [["", 0], ["", 0]]
berry_effects["ORANGE_BERRY"] = [["", 0], ["", 0]]
berry_effects["PINK_BERRY"] = [["", 0], ["", 0]]
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
def compute_euclidean_distance(x, z): return np.sqrt(x ** 2 + z ** 2)

class Vector:
    def __init__(self, distance, heading):
        self.distance = distance
        self.heading = heading

class Object:
    def __init__(self, distance, heading, risk_factor=0, attraction_factor=1):
        # placement of object relative to robot
        self.location_vector = Vector(distance, heading)
        self.update_risk(risk_factor, attraction_factor)
      
    def update_risk(self, risk_factor, attraction_factor):
        # risk factor assesses danger based on distance from object
        self.risk_factor = risk_factor
        # update risk_vecotr with new risk factor and attraction_factor = -1 or 1
        self.risk_vector = Vector(self.location_vector.distance * risk_factor, self.location_vector.heading * attraction_factor)
        
class Zombie(Object):
    def __init__(self, distance, heading):
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
    super().__init__(distance, heading)

class YellowBerry(Berry):
  def __init__(self, distance, heading):
    self.object_class_subtype = "YELLOW_BERRY"
    self.primary_effect = ""
    self.secondary_effect = ""
    super().__init__(distance, heading)

class OrangeBerry(Berry):
  def __init__(self, distance, heading):
    self.object_class_subtype = "ORANGE_BERRY"
    self.primary_effect = ""
    self.secondary_effect = ""
    super().__init__(distance, heading)

class PinkBerry(Berry):
  def __init__(self, distance, heading):
    self.object_class_subtype = "PINK_BERRY"
    self.primary_effect = ""
    self.secondary_effect = ""
    super().__init__(distance, heading)

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

##########################################################################################

cv2.startWindowThread()
cv2.namedWindow("HSV_PREVIEW")

# helper functions for extracting distance and heading given lidar information

def isbetween(num, lower, upper):
    return lower <= num <= upper
    
class ColorClassifiedObject:
    def __init__(self, center_x, center_y, width, height, camera):
        self.x = center_x
        self.y = center_y
        self.height = height
        self.width = width
        self.area = width * height
        self.camera = camera
        self.angle = self.transpose_pixel_to_angle(camera)
    """
        return: angle between [0, np.pi] to indicate direction of object
    """
    def transpose_pixel_to_angle(self, camera):
        pixel_offset = self.x - (camera.getWidth() / 2)
        theta = camera.getFov() / 2
        return np.arctan((2 * pixel_offset * np.tan(theta)) / camera.getWidth())

    """
        return: angle between [0, np.pi] to indicate direction of object
    """
    def __eq__(self, other):
        lower = 0
        upper = 20
        y_difference = isbetween(abs(self.y - other.y), lower, upper)
        width_difference = isbetween(abs(self.width - other.width), lower, upper)
        height_difference = isbetween(abs(self.height - other.height), lower, upper)
        area_difference = isbetween(abs(self.area - other.area), lower, upper ** 2)
        x_difference = isbetween(abs(abs(self.x - other.x) - self.width * 2), lower, upper) or isbetween(abs(self.x - other.x), lower, upper)
        if all((y_difference, width_difference, height_difference, area_difference, x_difference)):
            return True
        return False
        
class ImgProcessor:
    def __init__(self, camera):
        self.camera = camera
        self.height = camera.getHeight()
        self.width = camera.getWidth()
        self.berries = []

    def get_image(self):
        cameraData = self.camera.getImage()
        img = np.frombuffer(cameraData, np.uint8).reshape((self.height, self.width, 4))
        img = img[:, :, :3]
        return img

    def get_hsv_image(self):
        hsv_img = cv2.cvtColor(self.get_image(), cv2.COLOR_BGR2HSV)
        return hsv_img

    def filter_object_classes(self, hsv):
        # define range for red berries
        lower = np.array([0,100,0])
        upper = np.array([4,360,360])
        mask_red = cv2.inRange(hsv, lower, upper)
        # self.connected_components_analysis(hsv, mask_red)
        self.convert_classified_objects_to_objects(self.connected_components_analysis(hsv, mask_red), 'RED')
        # define range for orange berries
        lower = np.array([5,150,0])
        upper = np.array([15,360,360])
        mask_orange = cv2.inRange(hsv, lower, upper)
        # self.connected_components_analysis(hsv, mask_orange)
        self.convert_classified_objects_to_objects(self.connected_components_analysis(hsv, mask_orange), 'ORANGE')
        # define range for yellow berries
        lower = np.array([20,150,0])
        upper = np.array([30,360,360])
        mask_yellow = cv2.inRange(hsv, lower, upper)
        # self.connected_components_analysis(hsv, mask_yellow)
        self.convert_classified_objects_to_objects(self.connected_components_analysis(hsv, mask_yellow), 'YELLOW')

        # define range for pink berries
        lower = np.array([140,80,0])
        upper = np.array([170,360,360])
        mask_pink = cv2.inRange(hsv, lower, upper)
        # self.connected_components_analysis(hsv, mask_pink)
        self.convert_classified_objects_to_objects(self.connected_components_analysis(hsv, mask_pink), 'PINK')

    def connected_components_analysis(self, hsv, mask, filter_size=True, OTSU_THRESHOLD=False):
        color_classified_objects = []
        masked_image = cv2.bitwise_and(hsv, hsv, mask=mask)
        (h,s,v) = cv2.split(masked_image)
        img = cv2.merge((h, s, v))
        # convert to grayscale for connected component analysis
        img_converted = cv2.cvtColor(img, cv2.COLOR_HSV2RGB)	
        gray = cv2.cvtColor(img_converted, cv2.COLOR_RGB2GRAY)
        # otsu thresholding for background / foreground filtering
        if OTSU_THRESHOLD:
            img = cv2.threshold(img, 0, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)[1]
        # connected component analysis with 4 neighbor correlation
        (numLabels, labels, stats, centroids) = cv2.connectedComponentsWithStats(gray, 4, cv2.CV_32S)
        for i in range(0, len(centroids)):
            # extract the connected component statistics and centroid for
            # the current label
            w = stats[i, cv2.CC_STAT_WIDTH]
            h = stats[i, cv2.CC_STAT_HEIGHT]
            area = stats[i, cv2.CC_STAT_AREA]
            (cX, cY) = centroids[i]
            obj = ColorClassifiedObject(cX, cY, w, h, self.camera)

            # filter objects that are too big or too small
            keepWidth = 0 < w < 10000
            keepHeight = 0 < h < 10000
            keepArea = 10 < area < 10000
            if all((keepWidth, keepHeight, keepArea)):
                color_classified_objects.append(obj)
                # is obj similar to any other obj before it i.e. are we detecting two zombie legs as different components
                if filter_size:
                    for i, object in enumerate(color_classified_objects):
                        if object == obj:
                            # create new object whose bounding box and centroid encompass both objects
                            new_cX = (object.x + obj.x) / 2
                            new_cY = (object.y + obj.y) / 2
                            new_width = (object.width + obj.width) / 2 + abs(object.x - obj.x)
                            new_height = (object.height + obj.height) / 2 + abs(object.y - obj.y)
                            merged_object = ColorClassifiedObject(new_cX, new_cY, new_width, new_height, self.camera)

                            # remove the object from list
                            color_classified_objects.pop(i)
                            
                            color_classified_objects = color_classified_objects[:-1]
                            
                            # append new merged object
                            color_classified_objects.append(merged_object)
                            break 
        # display for debugging
        for object in color_classified_objects:
            output = hsv.copy()
            x, y = int(object.x + (object.width / 2)), int(object.y - (object.height / 2))
            xw, yw = int(object.x - (object.width / 2)), int(object.y + (object.height / 2))
            cv2.rectangle(output, (x, y), (xw, yw), (0, 255, 0), 3)
            cv2.circle(output, (int(object.x), int(object.y)), 4, (0, 0, 255), -1)
            print(object.angle)
            cv2.imshow("Classification", output)
            cv2.waitKey(0)
        return color_classified_objects
        
    def convert_classified_objects_to_objects(self, color_classified_object, color):
        for obj in color_classified_object:
            if color == 'YELLOW':
                self.berries.append(YellowBerry(1, obj.angle))
            if color == 'RED':
                self.berries.append(RedBerry(1, obj.angle))
            if color == 'ORANGE':
                self.berries.append(OrangeBerry(1, obj.angle))
            if color == 'PINK':
                self.berries.append(PinkBerry(1, obj.angle))
        
class LidarPoint:
    def __init__(self, x, y, z, layer_id=0, time=0):
        self.x = x
        self.y = y
        self.z = z
        self.layer_id = layer_id
        self.time = time
        self.heading = compute_heading(x, z)
        self.distance = compute_euclidian_distance(x, z)

    def __str__(self):
        return "x: {} y: {} z: {}".format(self.x, self.y, self.z)

class LidarProcessor:
    """

        :return:
    """
    def __init__(self, lidar):
        self.lidar = lidar
        self.lidar_point_cloud_history = []
        self.lidar_point_cloud = []

    def fetch_point_cloud(self):

        self.lidar_point_cloud = []
        lidarPoints_bytearray = self.lidar.getPointCloud(data_type='buffer')  # Get the point cloud byte array
        lidarPoints_list = self.lidar.getPointCloud(data_type='list')

        # unpack lidar points from buffer assuming there are 512 points
        lidarPoints = np.frombuffer(lidarPoints_bytearray, np.uint8).reshape(len(lidarPoints_list), 20)

        for point in lidarPoints:
            x = struct.unpack('f', point[0:4].tobytes())
            y = struct.unpack('f', point[4:8].tobytes())
            z = struct.unpack('f', point[8:12].tobytes())

            # filter unusable data
            if x not in [np.Inf, np.NaN, 0.0]:
                lidar_point = LidarPoint(x, y, z)
                self.lidar_point_cloud.append(lidar_point)

        # store history of point clouds in previous time steps
        self.lidar_point_cloud_history.append(self.lidar_point_cloud)
        if len(self.lidar_point_cloud_history) > 10:
            self.lidar_point_cloud_history = self.lidar_point_cloud_history[1:]

##########################################################################################
#pass in compass object, compass x and y before turning, targetAngle (-pi to pi), and wheel objects
def goToHeading(compass, startHeading, targetAngle, fr, fl, br, bl):
  current = [compass.getValues()[0], compass.getValues()[1]]
  dot_product = round(startHeading[0] * current[0] + startHeading[1] * current[1], 3)
  startMag = round(compute_euclidean_distance(startHeading[0], startHeading[1]), 3)
  currentMag = round(compute_euclidean_distance(current[0], current[1]), 3)
  angle = np.arccos(dot_product / (startMag * currentMag))
  speed = 14.81
  #if angle is more than 0, turn left
  if (targetAngle > 0):
    if not abs(angle - abs(targetAngle)) < 0.05:
      fr.setVelocity(-speed)
      bl.setVelocity(speed)
      fl.setVelocity(speed)
      br.setVelocity(-speed)
    else:
      fr.setVelocity(0.0)
      bl.setVelocity(0.0)
      fl.setVelocity(0.0)
      br.setVelocity(0.0) 
  else: # turn left
    if not abs(angle - abs(targetAngle)) < 0.05: 
      fr.setVelocity(speed)
      bl.setVelocity(-speed)
      fl.setVelocity(-speed)
      br.setVelocity(speed)
    else:
      fr.setVelocity(0.0)
      bl.setVelocity(0.0)
      fl.setVelocity(0.0)
      br.setVelocity(0.0) 
#--Helper--
# Returns closest object of given type
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

#--Helper--
# Update berry risk vectors based on berry dictionary values
def updateBerryRisks(objects):
    for item in objects:
        if item.object_class == "BERRY":
            berry_primary_effect = berry_effects[item.object_class_subtype][0][0]
            berry_secondary_effect = berry_effects[item.object_class_subtype][1][0]
            
            risk_factor = 0
            attraction_factor = 1
            
            if (berry_primary_effect == "+20 HEALTH"):
                risk_factor = 2
            elif (berry_primary_effect == "-20 ENERGY"):
                risk_factor = 3
                attraction_factor = -1
            elif (berry_primary_effect == "+40 ENERGY"):
                risk_factor = 4
            elif (berry_primary_effect == "+15 ARMOR"):
                risk_factor = 3
            
            if (berry_secondary_effect == "+20 HEALTH"):
                risk_factor += 0.5
            elif (berry_secondary_effect == "-20 ENERGY"):
                risk_factor += 0.75
                attraction_factor = -1
            elif (berry_secondary_effect == "+40 ENERGY"):
                risk_factor += 1
            elif (berry_secondary_effect == "+15 ARMOR"):
                risk_factor += 0.75
        
            item.update_risk(risk_factor, attraction_factor)


#--Helper--
# Updates the berry dictionary and modifies the berry's risk vector based on effects
def updateBerryDict(berry_color, berry_effect):
  first_effect_list = berry_effects[berry_color][0]
  first_effect = first_effect_list[0]
  second_effect_list = berry_effects[berry_color][1]
  second_effect = second_effect_list[0]

  if (berry_effect == first_effect):
    # increment first effect frequency
    berry_effects[berry_color][0][1] += 1
  elif (berry_effect == second_effect):
    # increment second effect frequency
    berry_effects[berry_color][1][1] += 1
  else:
    if first_effect == "":
       # effect is not found, just update the first effect data
      berry_effects[berry_color][0][0] = berry_effect
      berry_effects[berry_color][0][1] += 1
    elif second_effect == "":
      # effect is not found, update the second effect data
      berry_effects[berry_color][1][0] = berry_effect
      berry_effects[berry_color][1][1] += 1
    else:
      print("error more than 3 effects detected")

  # replace second effect with first effect if it is larger in frequency
  if berry_effects[berry_color][1][1] > berry_effects[berry_color][0][1]:
    temp = berry_effects[berry_color][0]
    berry_effects[berry_color][0] = berry_effects[berry_color][1]
    berry_effects[berry_color][1] = temp


#--Behavior--
#wander when no objects in sight
def wander(objects):
  heading, magnitude = 0, 0
  
  if len(objects) == 0:
    heading = np.random.normal(1, 0.25) * 3.141596
    magnitude = 0.2
  
  return heading, magnitude


#--Behavior--
# pursues the berry that is needed the most, or the closest berry
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

    if item.object_class == "BERRY":
        berry_primary_effect = berry_effects[item.object_class_subtype][0][0]
        berry_secondary_effect = berry_effects[item.object_class_subtype][1][0]
        
        if robot_health < 80 and robot_energy < 60:
          # only consume a known berry if you need it: health < 80 and energy < 60
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
        else:
          #--If health and energy are good, experiment on closest berries that do not have all their effects known yet --
          if item.location_vector.distance < distance and berry_secondary_effect == "":
            berry = item
            distance = item.location_vector.distance
  
  # If berry is found, change risk adjustments to move towards it
  if berry is not None:
    objects.remove(berry)
    berry.update_risk(2 * berry.location_vector.distance, berry.attraction_factor)
    objects.append(berry)
  
  return berry

#--Behavior--
#turns towards a detected solid (not zombie) in the lidar to classify it
def goToSolid(objects):
  target = None
  distance = 11
  #if there's no berries detected
  if checkForItem(objects, BERRY) is None:
    for item in objects:
      if item.object_class != ZOMBIE:
        if item.distance < distance: #get closest object
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
        item.update_risk(0.5 * item.distance * risk_multiplier, item.attraction_factor)
      elif item.object_class_subtype == "BLUE_ZOMBIE":
        # will relentlessly chase robot
        item.update_risk(0.3 * item.distance * risk_multiplier, item.attraction_factor)
      elif item.object_class_subtype == "AQUA_ZOMBIE":
        if item.distance < 4:
          # only generate when in 3m range
          item.update_risk(1 * item.distance * risk_multiplier, item.attraction_factor)
      elif item.object_class_subtype == "PURPLE_ZOMBIE":
        # If purple zombie, call helper func
        avoidPurpleZombie(objects, item, risk_multiplier)
      # else:
          # RUN

#--Behavior--
#pursue a berry in the opposite direction of purple zombie
def avoidPurpleZombie(objects, item, risk_multiplier):
  #check if there's a berry
  targetBerry = checkForItem(objects, BERRY)
  while targetBerry is not None and targetBerry.distance > 1:
    targetBerry.update_risk(item.distance * risk_multiplier)
    
  if (targetBerry.distance < 1):
    objects.remove(targetBerry)
    targetBerry.heading = np.pi / 2
    objects.append(targetBerry) 
    
  if (targetBerry is None):
    item.update_risk(1 * item.distance * risk_multiplier, item.attraction_factor)


def getResultantVector(objects):
  res_x = 0
  res_y = 0
  res_len = 0
  res_angle = 0

  for i in range(0, len(objects)):
    #print("vector distance: {}".format(objects[i].risk_vector.distance))
    new_len = objects[i].risk_vector.distance
    new_angle = objects[i].risk_vector.heading
 
    # x = L1 cos(a) + L2cos(a + b)
    temp_x = res_len * np.cos(res_angle) + new_len * np.cos(new_angle)
    # print("temp_x: {}".format(temp_x))
    # y = L1 sin(a) + L2sin(a + b)
    temp_y = res_len * np.sin(res_angle) + new_len * np.sin(new_angle)
    # print("temp_y: {}".format(temp_y))

    # Get resulting vector attributes
    res_x = temp_x
    res_y = temp_y
    res_len = np.sqrt(temp_x * temp_x + temp_y * temp_y)
    if (temp_x == 0 or temp_y == 0):
        res_angle = 0
    else:
        res_angle = np.arctan(temp_y/temp_x)
    
    # print("iteration {} has x: {} and y: {} and angle {}".format(i, res_x, res_y, res_angle))
    
    
    # if x is negative and y is positive, add 90 deg
    if (res_x < 0 and res_y > 0):
        # if x is negative and y is positive, add pi/2 deg
        res_angle += np.pi
    elif (res_x < 0 and res_y < 0):
        # if x and y are negative, add pi
        res_angle = np.pi + np.abs(res_angle)
    elif (res_x > 0 and res_y < 0):
        # if x is positive and y is negative, add 2pi
        res_angle += 2 * np.pi
    # if x and y is positive, do nothing
  
  # Change the last set of angles to -pi to pi
  # if x is negative and y is positive, add 90 deg
  if (res_x < 0 and res_y > 0):
      # if x is negative and y is positive, add pi/2 deg
      res_angle += np.pi
      res_angle *= -1
  elif (res_x < 0 and res_y < 0):
      # if x and y are negative, add pi
      res_angle = np.pi - np.abs(res_angle)
  elif (res_x > 0 and res_y < 0):
      # if x and y is positive, convert to negative
      res_angle *= -1
  # if x is positive and y is negative, do nothing

  return Vector(res_len, res_angle)


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
    
    camera3 = robot.getDevice("ForwardHighRes")
    camera3.enable(timestep)
    
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
    startTurnHeading = [None, None]
    
    prev_health = 100
    prev_energy = 100
          

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
     

              # **** PART 1: Intialize variables ****
        processor = ImgProcessor(camera3)
        image = processor.get_image()
        hsv = processor.get_hsv_image()
    
        processor.filter_object_classes(hsv)
        object_list = processor.berries
        
        # for obj in object_list:
            # print("obj distance: {}".format(obj.risk_vector.distance))
            # print("obj heading: {}".format(obj.risk_vector.heading)) 
        
        #called every timestep
        curr_health = robot_info[0]
        curr_energy = robot_info[1]
        curr_armor = robot_info[2]
        
        
        # **** PART 2: Use Motor Schema to create resultant vector from behaviors ****
        # avoidZombie(object_list, robot_info)
        temp_berry = pursueBerry(object_list, robot_info)
        if (temp_berry is not None):
            last_berry = temp_berry
        else:
            if (i % 25  == 0):
                last_berry = temp_berry
        print(last_berry)
        # goToSolid(object_list)
        # wander(object_list)
        res_vector = getResultantVector(object_list)
        # print("res_vector heading: {}".format(res_vector.heading))
        
        #go forward indefinitely --> change maybe to stop and turn for bigger angles
        
        fr.setVelocity(14.8)
        bl.setVelocity(14.8)
        fl.setVelocity(14.8)
        br.setVelocity(14.8) 
        
        # **** PART 3: Classify Berries ****
        # previous berry can only have 1 effect
        
        if (curr_health - prev_health >= 19 or curr_health >= 99 and prev_health < 99):
          # classify last berry as +20 HEALTH
          print("+20 HEALTH")
          if last_berry:
              updateBerryDict(last_berry.object_class_subtype, "+20 HEALTH")
        elif(curr_energy - prev_energy <= -19):
          # classify last berry as -20 ENERGY
          print("-20 ENERGY")
          if last_berry:
              updateBerryDict(last_berry.object_class_subtype, "-20 ENERGY")
        elif(curr_energy - prev_energy >= 39 or curr_energy >= 99 and prev_energy < 99):
          # classify last berry as +40 ENERGY
          print("+40 ENERGY")
          if last_berry:
              updateBerryDict(last_berry.object_class_subtype, "+40 ENERGY")
          print(berry_effects)
        elif(curr_armor >= 14):
          # classify last berry as +15 ARMOR
          print("+15 ARMOR")
          if last_berry:
              updateBerryDict(last_berry.object_class_subtype, "+15 ARMOR")
        

        # **** PART 4: Send resultant vector data to actuators ****
        if (i % 4 == 0):
          #need to update the starting absolute heading before each turn
          startTurnHeading[0], startTurnHeading[1] = compass.getValues()[0], compass.getValues()[1]
          #execute the turn
          goToHeading(compass, startTurnHeading, res_vector.heading, fr, fl, br, bl)
        

        # **** PART 5: Update berry risk vectors ****
        updateBerryRisks(object_list)
        # print(berry_effects)
        if (i % 16 == 0):
            # print("prev_energy: {} curr_energy:{}".format(prev_energy, curr_energy))
            prev_health = curr_health
            prev_energy = curr_energy
        #update count of while loop
        i += 1
        #------------------CHANGE CODE ABOVE HERE ONLY--------------------------
        
        
    return 0   


main()
