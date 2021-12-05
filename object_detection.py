"""youbot_controller controller."""

from controller import Robot, Motor, Camera, Accelerometer, GPS, Gyro, LightSensor, Receiver, RangeFinder, Lidar
from controller import Supervisor

from youbot_zombie import *
import numpy as np
import sys
import cv2
import struct


# ------------------CHANGE CODE BELOW HERE ONLY--------------------------
# define functions here for making decisions and using sensor inputs

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
        lower = np.array([0,110,0])
        upper = np.array([5,360,260])
        mask_red = cv2.inRange(hsv, lower, upper)
        self.convert_classified_objects_to_objects(self.connected_components_analysis(hsv, mask_red), 'RED')
        # define range for orange berries
        lower = np.array([5,110,0])
        upper = np.array([15,360,360])
        mask_orange = cv2.inRange(hsv, lower, upper)
        self.convert_classified_objects_to_objects(self.connected_components_analysis(hsv, mask_orange), 'ORANGE')
        # define range for yellow berries
        lower = np.array([20,110,0])
        upper = np.array([30,360,360])
        mask_yellow = cv2.inRange(hsv, lower, upper)
        self.convert_classified_objects_to_objects(self.connected_components_analysis(hsv, mask_yellow), 'YELLOW')

        # define range for pink berries
        lower = np.array([140,80,0])
        upper = np.array([170,360,360])
        mask_pink = cv2.inRange(hsv, lower, upper)
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
        