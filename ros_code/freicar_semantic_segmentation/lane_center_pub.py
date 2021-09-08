import argparse
import torch
import torch.nn.parallel
import torch.optim
import torch.utils.data
import torch.utils.data.distributed
from model import fast_scnn_model

import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2
import numpy as np
import torchvision.transforms.functional as TF

import os
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
import birdsEyeT
import random


a = os.getcwd()
parser = argparse.ArgumentParser(description='Segmentation and Regression Training')
parser.add_argument('--resume', default=a+'/checkpoint.pth.tar', type=str, metavar='PATH',
                    help='path to latest checkpoint (default: none)')
parser.add_argument('--start_epoch', default=0, type=int, help="Start at epoch X")
parser.add_argument('--batch_size', default=1, type=int, help="Batch size for training")
parser.add_argument('-e', '--evaluate', dest='evaluate', action='store_true',
                    help='evaluate model on validation set')
parser.add_argument('--print-freq', '-p', default=10, type=int,
                    metavar='N', help='print frequency (default: 10)')

best_iou = 0
args = parser.parse_args()
model = fast_scnn_model.Fast_SCNN(3, 4)
model = model.cuda()
model.train()

pub = rospy.Publisher('/freicar_1/seg_classes', Image, queue_size=10)
pub2 = rospy.Publisher('/freicar_1/seg_lanes', Image, queue_size=10)
pub1 = rospy.Publisher('/freicar_1/bev', Image, queue_size=10)
topic = 'freicar_1/lane_center_points'
pub3 = rospy.Publisher(topic, MarkerArray, queue_size=10)


markerArray = MarkerArray()

count = 0
MARKERS_MAX = 10000

if args.resume:
    if os.path.isfile(args.resume):
        print("=> loading checkpoint '{}'".format(args.resume))
        model.load_state_dict(torch.load(args.resume)['state_dict'])
        args.start_epoch = 0


    else:
        print("=> no checkpoint found at '{}'".format(args.resume))
else:
    args.start_epoch = 0

def visJetColorCoding(img):
    color_img = np.zeros(img.shape, dtype=img.dtype)
    cv2.normalize(img, color_img, 0, 255, cv2.NORM_MINMAX)
    color_img = color_img.astype(np.uint8)
    color_img = cv2.applyColorMap(color_img, cv2.COLORMAP_JET, color_img)
    return color_img

def TensorImage1ToCV(data):
   cv = data.cpu().data.numpy().squeeze()
   return cv


def callback(msg):

    np_img = np.fromstring(msg.data, dtype=np.uint8).reshape((720, 1280, 3))
    img_tensor = TF.to_tensor(np_img)
    img_tensor = TF.resize(img_tensor, [384, 640])                 # shape is now (384,640)
    img_tensor.unsqueeze_(0)
    img_tensor = img_tensor.cuda()

    with torch.no_grad():
        seg_cla, seg_reg = model(img_tensor)

    pred = torch.argmax(seg_cla, 1)

    bridge = CvBridge()

    #semantic segmentation
    color_img = visJetColorCoding(TensorImage1ToCV(pred[0].float()))
    image_message = bridge.cv2_to_imgmsg(color_img)

    #lane regression
    seg_lanes = TensorImage1ToCV(seg_reg)
    color_lanes = visJetColorCoding(seg_lanes)
    lanes_msg = bridge.cv2_to_imgmsg(color_lanes)

    #publish semantic segmentation and lane regression
    pub.publish(image_message)
    pub2.publish(lanes_msg)

    #bev
    hom_conv = birdsEyeT.birdseyeTransformer('dataset_helper/freicar_homography.yaml', 3, 3, 200, 2)            # 3mX3m, 200 pixel per meter
    bev_lanes = visJetColorCoding(hom_conv.birdseye(seg_lanes))                                                 # 600x600
    bev_msg = bridge.cv2_to_imgmsg(bev_lanes)

    #publish bev
    pub1.publish(bev_msg)

    #threshold bev
    lanes_gray = cv2.cvtColor(bev_lanes, cv2.COLOR_RGB2GRAY)
    #cv2.imshow("gray", lanes_gray)
    #print(lanes_gray)
    cv2.waitKey(1)
    lane_thres = np.argwhere(lanes_gray < 38)

    #marker array

    N_max = 10
    lanes_thres_sample = random.choices(lane_thres, k=N_max)

    counter = 0

    for x, y in lanes_thres_sample:

        x_n = x / 200.0
        y_n = y / 200.0 - bev_lanes.shape[0]/400.0

        marker = Marker()
        marker.header.frame_id = "freicar_1/base_link"
        marker.header.stamp = rospy.Time.now()
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.pose.position.x = x_n
        marker.pose.position.y = y_n
        markerArray.markers.append(marker)

        counter += 1
        if (counter > N_max):
            markerArray.markers.pop(0)

    id = 0
    for n in markerArray.markers:
        n.id = id
        id += 1

    #publish marker array
    pub3.publish(markerArray)



def subscriber_publisher():
    rospy.init_node('seg_reg_publisher', anonymous=True)
    img_sub = rospy.Subscriber('/freicar_1/sim/camera/rgb/front/image', Image, callback, queue_size=10)
    rospy.spin()



if __name__ == '__main__':
    subscriber_publisher()