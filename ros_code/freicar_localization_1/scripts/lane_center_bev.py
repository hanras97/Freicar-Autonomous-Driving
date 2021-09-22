import argparse
import torch
import torch.nn.parallel
import torch.optim
import torch.utils.data
import torch.utils.data.distributed
from std_msgs.msg import Header

import os
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2
import numpy as np
import torchvision.transforms.functional as TF
from model import fast_scnn_model
import birdsEyeT


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

pub = rospy.Publisher('/freicar_1/sim/camera/rgb/front/reg_bev', Image, queue_size=10)

count = 0
if args.resume:
    if os.path.isfile(args.resume):
        print("=> loading checkpoint '{}'".format(args.resume))
        model.load_state_dict(torch.load(args.resume)['state_dict'])
        args.start_epoch = 0


    else:
        print("=> no checkpoint found at '{}'".format(args.resume))
else:
    args.start_epoch = 0

def thresholdImage(img, t):
    torch_thresholding = torch.nn.Threshold(t, 0)
    return torch_thresholding(img)


def visJetColorCoding(img):
    color_img = np.zeros(img.shape, dtype=img.dtype)
    cv2.normalize(img, color_img, 0, 255, cv2.NORM_MINMAX)
    color_img = color_img.astype(np.uint8)
    color_img = cv2.applyColorMap(color_img, cv2.COLORMAP_JET, color_img)
    return color_img

def TensorImage1ToCV(data):
    cv = data.cpu().data.numpy().squeeze()
    return cv

def visImage3Chan(data):
    cv = np.transpose(data.cpu().data.numpy().squeeze(), (1, 2, 0))
    cv = cv2.cvtColor(cv, cv2.COLOR_RGB2BGR)
    # cv2.imshow(name, cv)

def callback(msg):

    header = Header(stamp=rospy.Time.now())
    header.frame_id = 'freicar_1/base_link'
    msg.header = header
    np_img = np.fromstring(msg.data, dtype=np.uint8).reshape((720, 1280, 3))
    #np_img = np_img[:, :, :3]
    img_tensor = TF.to_tensor(np_img)
    img_tensor = TF.resize(img_tensor,[384,640])
    img_tensor.unsqueeze_(0)
    img_tensor = img_tensor.cuda()                                   # np.shape torch.Size([1, 3, 384, 640])

    hom_conv = birdsEyeT.birdseyeTransformer('freicar_homography.yaml', 3, 3, 200, 2)

    with torch.no_grad():
        seg_cla, seg_reg = model(img_tensor)                             # np.shape torch.Size([1, 4, 384, 640])

    thresh = 1
    bridge = CvBridge()
    seg_reg = seg_reg.squeeze()

    # if (torch.nonzero(seg_reg).shape[0]>= 0):
    #     seg_reg_thresh = TensorImage1ToCV(seg_reg)
    #     seg_reg_thresh_bev = hom_conv.birdseye(seg_reg_thresh)

    seg_lanes = TensorImage1ToCV(seg_reg)
    #bev_lanes = np.zeros(seg_lanes.shape, dtype=seg_lanes.dtype)
    #cv2.normalize(seg_lanes, bev_lanes, 0, 255, cv2.NORM_MINMAX)
    bev_lanes = hom_conv.birdseye(seg_lanes)
    ret, thresh_img = cv2.threshold(bev_lanes, thresh, 255, cv2.THRESH_BINARY)

    #lanes_gray = cv2.cvtColor(bev_lanes, cv2.COLOR_RGB2GRAY)
    #print(np.sum(thresh_img==255))


    #cv2.imshow("image", thresh_img)
    #print(np.max(thresh_img))

    lanes_msg = bridge.cv2_to_imgmsg(thresh_img)

    lanes_msg.header = header
    lanes_msg.header.frame_id = 'freicar_1/base_link'
    # seg_reg_thresh_bev = TF.to_tensor(seg_reg_thresh_bev)
    # bev_thresh = thresholdImage(seg_reg_thresh_bev, thresh)
    # print(seg_reg_thresh_bev)
    # bev_thresh = bev_thresh.unsqueeze_(0)
    # bev_thresh = bev_thresh.numpy()
    # bev_thresh = bev_thresh[0, 0, :, :]
    #
    #
    #
    #
    # image_message = bridge.cv2_to_imgmsg(bev_thresh)
    # image_message.header = header
    # image_message.header.frame_id = 'freicar_1/base_link'
    #cv2.imshow("mat",thresh_img)
    #print(bev_lanes)
    pub.publish(lanes_msg)



def subscriber_publisher():
    rospy.init_node('bev_lanes_img', anonymous=True)
    img_sub = rospy.Subscriber('/freicar_1/sim/camera/rgb/front/image', Image, callback, queue_size=10)
    rospy.spin()



if __name__ == '__main__':
    subscriber_publisher()