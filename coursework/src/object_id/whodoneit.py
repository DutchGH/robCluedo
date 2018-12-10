#!/usr/bin/env python
# This final piece fo skeleton code will be centred around gettign the students to follow a colour and stop upon sight of another one.

from __future__ import division
import cv2
import numpy as np
import rospy
import sys
import tensorflow as tf

from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError

class CleudoCharacter:
    def __init__(self, name, fn):
        self.name = name
        self.fn = fn
        self.convScore = 0
        self.templateScore = 0

    def getScore(self):
        return self.templateScore

    def setScore(self, score):
        self.templateScore = score

class colourIdentifier():

    def __init__(self, pub, rate, graph, labels):
        # Initialise a publisher to publish messages to the robot base
        # We covered which topic receives messages that move the robot in the 2nd Lab Session
        self.publisher = pub
        self.rate = rate
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.callback)
        

        #Vars for image detection - courtesy of Andy Bullpit (School of Computing)
        self.input_height = 224
        self.input_width = 224
        self.input_mean = 0
        self.input_std = 255
        self.input_layer = "Placeholder"
        self.output_layer = "final_result"

        self.graph = graph
        self.labels = labels

        self.input_name = "import/" + self.input_layer
        self.output_name = "import/" + self.output_layer
        self.input_op = self.graph.get_operation_by_name(self.input_name)
        self.output_op = self.graph.get_operation_by_name(self.output_name)
        



        # Initialise any flags that signal a colour has been detected in view


        # Initialise the value you wish to use for sensitivity in the colour detection (10 should be enough)


        # Initialise some standard movement messages such as a simple move forward and a message with all zeroes (stop)


    def callback(self, data):
        clu_list = createCharacterList()
 
        
        #Conv NN - Not having too much success with this if I'm honest
        # with tf.Session(graph = self.graph) as sess:
        #     try:
        #         # Convert the received image into a opencv image
        #         cv_image = self.bridge.imgmsg_to_cv2(data, "rgb8")
        #     except CvBridgeError as e:
        #         print(e)

        #     hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        #     gs = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
        #     resized_frame = cv2.resize(gs, (self.input_height, self.input_width), interpolation=cv2.INTER_AREA)
        #     # cv2.namedWindow('Camera_Feed')
        #     # cv2.imshow('Camera_Feed', resized_frame)
        #     numpy_frame = np.float32(resized_frame)
        #     normalised = cv2.normalize(numpy_frame, None, alpha = 0, beta=1, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_32F)
        #     cv2.namedWindow('WINDOW2')
        #     cv2.imshow('WINDOW2', normalised)
        #     t = np.expand_dims(normalised, axis = 0)

        #     results = sess.run(self.output_op.outputs[0],{self.input_op.outputs[0]: t})
        #     results = np.squeeze(results)
        #     top_k = results.argsort()[-7:][::-1]
        #     print(self.labels[top_k[0]], results[top_k[0]])
        #     # loc = np.where(results[top_k[0]] >= 0.2)
        #     # print(loc)

        #     cv2.waitKey(3)
        #         # if cv2.waitKey(1) & 0xFF == ord('q'):
        #         #     sess.close()
        #         #     break
        
        #........ TEMPLATE MATCHING ....................#
        bestCharacter = CleudoCharacter(None, None)
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        conv = np.float32(cv_image)
        img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        img2 = img.copy()
        for clu in clu_list:
            template = cv2.imread(clu.fn,0)
            template = cv2.resize(template, None, fx = 0.3, fy = 0.3, interpolation = cv2.INTER_CUBIC)
            w, h = template.shape[::-1]

            meth = 'cv2.TM_CCOEFF_NORMED'

            img = img2.copy()
            method = eval(meth)

            # Apply template Matching
            res = cv2.matchTemplate(img,template,method)
            min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
            clu.setScore(max_val)
            if clu.getScore() > bestCharacter.getScore() and clu.getScore() > 0.5:
                bestCharacter = clu


        cv2.namedWindow('WINDOW2')
        cv2.imshow('WINDOW2', img)
        print (bestCharacter.name, bestCharacter.getScore())
        cv2.waitKey(3)
        

def publisher():
    pub = rospy.Publisher('Cluedo_Result', String, queue_size=10)
    rate = rospy.Rate(10) #10hz

    return pub, rate

def load_graph(model_file):
    graph = tf.Graph()
    graph_def = tf.GraphDef()

    with open(model_file, "rb") as f:
        graph_def.ParseFromString(f.read())
    with graph.as_default():
        tf.import_graph_def(graph_def)

    return graph


def load_labels(label_file):
    label = []
    proto_as_ascii_lines = tf.gfile.GFile(label_file).readlines()
    for l in proto_as_ascii_lines:
        label.append(l.rstrip())
    return label


def createCharacterList():
    clu_list = []
    clu_list.append(CleudoCharacter("Mustard", "templates/mustard.png"))
    clu_list.append(CleudoCharacter("Peacock", "templates/peacock.png"))
    clu_list.append(CleudoCharacter("Scarlet", "templates/scarlet.png"))
    clu_list.append(CleudoCharacter("Plum", "templates/plum.png"))
    clu_list.append(CleudoCharacter("Wrench", "templates/wrench.png"))
    clu_list.append(CleudoCharacter("Rope", "templates/rope.png"))
    clu_list.append(CleudoCharacter("Revolver", "templates/scarlet.png"))

    return clu_list
    
# Create a node of your class in the main and ensure it stays up and running
# handling exceptions and such
def main(args):
    rospy.init_node('cluedo_detector', anonymous=True)
    pub, rate = publisher()
    model_file = "final_graph.pb"
    label_file = "output_labels.txt"

    graph = load_graph(model_file)
    labels = load_labels(label_file)
    cI = colourIdentifier(pub, rate, graph, labels)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

# Check if the node is executing in the main path
if __name__ == '__main__':
    main(sys.argv)
