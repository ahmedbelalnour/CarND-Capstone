from styx_msgs.msg import TrafficLight
import numpy as np
import cv2
import os

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
	"""self.recorded_imgs_directory = '../../../recorded_imgs_classifier/'
	if not os.path.exists(self.recorded_imgs_directory):
		os.makedirs(self.recorded_imgs_directory)"""

    def get_classification(self, image):
        """Determines the color of the traffic light in the image
        Args:
            image (cv2 image): image containing the traffic light
        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """

	ym = cv2.inRange(image, np.array([0,150,150], dtype = "uint8"), np.array([62,255,255], dtype = "uint8"))
	rm = cv2.inRange(image, np.array([17,15,100], dtype = "uint8"), np.array([50,56,255], dtype = "uint8"))
	gm = cv2.inRange(image, np.array([0,100,0], dtype = "uint8"), np.array([50,255,56], dtype = "uint8"))

	output_yellow = cv2.bitwise_and(image, image, mask = ym)
	output_red = cv2.bitwise_and(image, image, mask = rm)
	output_green = cv2.bitwise_and(image, image, mask = gm)

	num_yellow_pixels = np.sum((output_yellow > 150) == 1)
	num_red_pixels = np.sum((output_red > 100) == 1)
	num_green_pixels = np.sum((output_green > 100) == 1)
	
	maximum_pixels = max(num_red_pixels, num_yellow_pixels, num_green_pixels)
	traffic_state = TrafficLight.UNKNOWN
	if(num_red_pixels == maximum_pixels and num_red_pixels > 300):
	    print("traffic red")
	    traffic_state = TrafficLight.RED	    
        elif(num_yellow_pixels > 120 and num_yellow_pixels == maximum_pixels):
	    print("traffic yellow")
	    traffic_state = TrafficLight.YELLOW
        elif num_green_pixels > 8:
	    print("traffic green")
            traffic_state = TrafficLight.GREEN
	elif traffic_state == TrafficLight.UNKNOWN:
	    print("Red recovery")
	    traffic_state = TrafficLight.RED
        return traffic_state
