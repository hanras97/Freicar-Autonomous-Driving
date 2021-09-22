#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
import cv2

import torch
import yaml
import sys

sys.path.insert(0,'/home/freicar/freicar_ws/src/freicar_object_bb/src/')
from model.efficientdet.backbone import EfficientDetBackbone
from model.efficientdet.utils import BBoxTransform, ClipBoxes
from utils import postprocess, boolean_string
from dataloader.freicar_dataloader import FreiCarDataset
from model.efficientdet.dataset import collater
from torch.utils.data import DataLoader
import numpy as np

# from mean_average_precision import MetricBuilder

from std_msgs.msg import String
import torchvision.transforms.functional as TF
from std_msgs.msg import Float32MultiArray as bb


compound_coef = 0
nms_threshold = 0.5
use_cuda = True
gpu = 0


project_name = 'freicar-detection'
weights_path = '/home/freicar/freicar_ws/src/freicar_object_bb/src/logs/freicar-detection/efficientdet-d0_99_109100.pth'
import os, sys
here = os.path.dirname(os.path.abspath('/home/freicar/freicar_ws/src/freicar_object_bb/src/projects/'))
sys.path.append(here)

filename = os.path.join(here, "projects/freicar-detection.yml")
params = yaml.safe_load(open(filename))
obj_list = params['obj_list']

threshold = 0.2
iou_threshold = 0.5

# test_image = Image.open(test_image_name).convert('RGB')test_image = Image.open(test_image_name).convert('RGB')
def callback(msg):
    np_img = np.fromstring(msg.data, dtype=np.uint8).reshape((720, 1280, 3))
    np_img = np_img[:, :, :3]
    img_tensor = TF.to_tensor(np_img)

    img_tensor = TF.resize(img_tensor,[384,640])

    np_img2 = img_tensor.numpy()
    np_img2 = np.resize(np_img2, (384, 640))

    img_tensor.unsqueeze_(0)
    img_tensor = img_tensor.cuda()

    model = EfficientDetBackbone(compound_coef=compound_coef, num_classes=len(obj_list),ratios=eval(params['anchors_ratios']), scales=eval(params['anchors_scales']))

    try:
        model.load_state_dict(torch.load(weights_path, map_location='cpu'))
    except:
        checkpoint = torch.load(weights_path, map_location=torch.device('cpu'))
        checkpoint = {key.replace("model.", ""): value for key, value in checkpoint.items()}
        checkpoint = {key.replace("backbone_net.", "backbone_net.model."): value for key, value in checkpoint.items()}
        model.load_state_dict(checkpoint)

    model.requires_grad_(False)
    model.eval()

    if use_cuda:
        model = model.cuda()

    with torch.no_grad():
        features, regression, classification, anchors = model(img_tensor)

    regressBoxes = BBoxTransform()
    clipBoxes = ClipBoxes()
    preds = postprocess(img_tensor, anchors, regression, classification, regressBoxes, clipBoxes, threshold,nms_threshold)
    preds = preds[0]
    
    bb_msg = preds['rois']

    if(preds['scores'][0] > 0.50 ):
        bb_msg = preds['rois']
    else:
        bb_msg = preds['rois']
        bb_msg[0][0] = 0.0
        bb_msg[0][1] = 0.0
        bb_msg[0][2] = 0.0
        bb_msg[0][3] = 0.0

    pub = rospy.Publisher('/freicar_1/bounding_box', bb, queue_size=10)

    rate = rospy.Rate(10)  # 10hz

    msg_to_publish = bb()
    msg_to_publish.data = np.array([bb_msg[0][0], bb_msg[0][1], bb_msg[0][2], bb_msg[0][3]])
    cv2.rectangle(np_img2, (msg_to_publish.data[0].astype(int),msg_to_publish.data[1].astype(int)),(msg_to_publish.data[2].astype(int),msg_to_publish.data[3].astype(int)), (0, 255, 0), -1)
    print("bounding boxes  = ", msg_to_publish.data[0], msg_to_publish.data[1],msg_to_publish.data[2], msg_to_publish.data[3])
    pub.publish(msg_to_publish)
    rate.sleep()

def subscriber_publisher():
    rospy.init_node('rospubsub', anonymous=True)
    img_sub = rospy.Subscriber('/freicar_1/sim/camera/rgb/front/image', Image, callback, queue_size=10)
    rospy.spin()


if __name__ == '__main__':

    subscriber_publisher()
