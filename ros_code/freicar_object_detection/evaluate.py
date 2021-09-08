"""
COCO-Style Evaluations
"""

import json
import os

import argparse
import torch
import yaml
from torchvision.ops import box_iou
from mean_average_precision import MetricBuilder, mean_average_precision_2d
from tqdm import tqdm
from model.efficientdet.backbone import EfficientDetBackbone
from model.efficientdet.utils import BBoxTransform, ClipBoxes
from utils import *
from dataloader.freicar_dataloader import FreiCarDataset
from model.efficientdet.dataset import collater
from torch.utils.data import DataLoader
import matplotlib.pyplot as plt

import numpy as np


########################################################################
# Object Detection model evaluation script
# Modified by: Jannik Zuern (zuern@informatik.uni-freiburg.de)
########################################################################


ap = argparse.ArgumentParser()
ap.add_argument('-p', '--project', type=str, default='freicar-detection', help='project file that contains parameters')
ap.add_argument('-c', '--compound_coef', type=int, default=0, help='coefficients of efficientdet')
ap.add_argument('-w', '--weights', type=str, default=None, help='/path/to/weights')
ap.add_argument('--nms_threshold', type=float, default=0.5, help='nms threshold, don\'t change it if not for testing purposes')
ap.add_argument('--cuda', type=boolean_string, default=True)
ap.add_argument('--device', type=int, default=0)
args = ap.parse_args()


compound_coef = args.compound_coef
nms_threshold = args.nms_threshold
use_cuda = args.cuda
gpu = args.device

project_name = args.project
weights_path = args.weights


params = yaml.safe_load(open(f'projects/{project_name}.yml'))
obj_list = params['obj_list']

threshold = 0.2
iou_threshold = 0.5

if __name__ == '__main__':

    metric_fn = MetricBuilder.build_evaluation_metric("map_2d", async_mode=True, num_classes=1)
    '''
    Note: 
    When calling the model forward function on an image, the model returns
    features, regression, classification and anchors.
    
    In order to obtain the final bounding boxes from these predictions, they need to be postprocessed
    (this performs score-filtering and non-maximum suppression)
    
    Thus, you should call
    

    regressBoxes = BBoxTransform()
    clipBoxes = ClipBoxes()

    preds = postprocess(imgs, anchors, regression, classification, regressBoxes, clipBoxes, threshold, nms_threshold)                  
    preds = preds[0]

    Now, the scores, class_indices and bounding boxes are saved as fields in the preds dict and can be used for subsequent evaluation.
    '''

    set_name = 'validation'

    freicar_dataset = FreiCarDataset(data_dir="./dataloader/data/",
                                     padding=(0, 0, 12, 12),
                                     split=set_name,
                                     load_real=False)
    val_params = {'batch_size': 1,
                  'shuffle': False,
                  'drop_last': True,
                  'collate_fn': collater,
                  'num_workers': 1}

    freicar_generator = DataLoader(freicar_dataset, **val_params)

    # instantiate model
    model = EfficientDetBackbone(compound_coef=compound_coef,
                                     num_classes=len(obj_list),
                                     ratios=eval(params['anchors_ratios']),
                                     scales=eval(params['anchors_scales']))
    
    # load model weights file from disk
    try:
        model.load_state_dict(torch.load(weights_path, map_location='cpu'))
    except:
        checkpoint = torch.load(weights_path, map_location=torch.device('cpu'))
        checkpoint = {key.replace("model.",""): value for key, value in checkpoint.items()}
        checkpoint = {key.replace("backbone_net.", "backbone_net.model."): value for key, value in checkpoint.items()}
        model.load_state_dict(checkpoint)

    model.requires_grad_(False)
    model.eval()
    if use_cuda:
        model = model.cuda()

    miou = []
    miou_new = []
    miou_threshold = []

    for iter, data in enumerate(freicar_generator):
        
        imgs = data['img'].cuda()
        annot = data['annot'].cuda()
        with torch.no_grad():
            features, regression, classification, anchors = model(imgs)
        
        regressBoxes = BBoxTransform()
        clipBoxes = ClipBoxes()
        input_annot = []
        output_annot = []
        bbox_annotation = annot[0]
        
        preds = postprocess(imgs, anchors, regression, classification, regressBoxes, clipBoxes, threshold,
                            nms_threshold)
        preds = preds[0]
        
        roi = torch.from_numpy(preds['rois'])
        rois2 = roi.cuda()

        if torch.numel(rois2) != 0 and annot[0][0, :4][0] != -1:
            iou = box_iou(rois2, annot[0][:, :4])
            miou.append(iou.tolist())

        for k in annot:
            for l in k:
                l = l.tolist()
                l.append(0)
                l.append(0)
                input_annot.append(l)

        scores = preds['scores']
        class_ids = preds['class_ids']
        rois = preds['rois']

        for roi_id in range(rois.shape[0]):
            score = float(scores[roi_id])
            label = int(class_ids[roi_id])
            box = rois[roi_id, :].tolist()
            pred_out = box
            pred_out.append(label)
            pred_out.append(score)
            output_annot.append(pred_out)

        output_annot = np.array(output_annot)
        input_annot = np.array(input_annot)
        metric_fn.add(output_annot, input_annot)

    # compute metric COCO metric
    print(f"mAP: {metric_fn.value(iou_thresholds=0.5, recall_thresholds=np.arange(0.0, 1.01, 0.01), mpolicy='soft')['mAP']}")

    # compute mIoU
    for x in miou:
        for y in x:
            miou_new.append(y)

    for x in miou_new:
        for y in x:
            if float(y) > 0.5:
                miou_threshold.append(y)

    print(f"mIoU: {np.mean(miou_threshold)}")

    # plot precision-recall curve
    precision = metric_fn.value(iou_thresholds=0.5, recall_thresholds=np.arange(0.0, 1.01, 0.01), mpolicy='soft')[iou_threshold][0]['precision']
    recall = metric_fn.value(iou_thresholds=0.5, recall_thresholds=np.arange(0.0, 1.01, 0.01), mpolicy='soft')[iou_threshold][0]['recall']
    plt.plot(recall, precision)
    plt.xlabel('Recall')
    plt.ylabel('Precision')
    plt.title('Precision-Recall Curve')
    plt.show()





