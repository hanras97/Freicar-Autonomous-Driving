import argparse
import os
import shutil
import time

import torch
import torch.nn.parallel
import torch.optim
import torch.utils.data
import torch.utils.data.distributed

import cv2
import numpy as np
from torch.optim import SGD, Adam, lr_scheduler

from model import fast_scnn_model
from dataset_helper import freicar_segreg_dataloader
from dataset_helper import color_coder
from ioumetric import SegmentationMetric

#################################################################
# AUTHOR: Johan Vertens (vertensj@informatik.uni-freiburg.de)
# DESCRIPTION: Training script for FreiCAR semantic segmentation
##################################################################

def visJetColorCoding(name, img):
    img = img.detach().cpu().squeeze().numpy()
    color_img = np.zeros(img.shape, dtype=img.dtype)
    cv2.normalize(img, color_img, 0, 255, cv2.NORM_MINMAX)
    color_img = color_img.astype(np.uint8)
    color_img = cv2.applyColorMap(color_img, cv2.COLORMAP_JET, color_img)
    cv2.imshow(name, color_img)

def visImage3Chan(data, name):
    cv = np.transpose(data.cpu().data.numpy().squeeze(), (1, 2, 0))
    cv = cv2.cvtColor(cv, cv2.COLOR_RGB2BGR)
    cv2.imshow(name, cv)


parser = argparse.ArgumentParser(description='Segmentation and Regression Training')
parser.add_argument('--resume', default='', type=str, metavar='PATH',
                    help='path to latest checkpoint (default: none)')
parser.add_argument('--start_epoch', default=0, type=int, help="Start at epoch X")
parser.add_argument('--batch_size', default=10, type=int, help="Batch size for training")
parser.add_argument('-e', '--evaluate', dest='evaluate', action='store_true',
                    help='evaluate model on validation set')
parser.add_argument('--print-freq', '-p', default=10, type=int,
                    metavar='N', help='print frequency (default: 10)')
best_iou = 0
args = None


def main():
    global args, best_iou
    args = parser.parse_args()

    # Create Fast SCNN model...
    model = fast_scnn_model.Fast_SCNN(3, 4)
    model = model.cuda()

    # Number of max epochs, TODO: Change it to a reasonable number!
    num_epochs = 6

    optimizer = Adam(model.parameters(), 5e-3)
    lambda1 = lambda epoch: pow((1 - ((epoch - 1) / num_epochs)), 0.9)
    scheduler = lr_scheduler.LambdaLR(optimizer, lr_lambda=lambda1)

    # optionally resume from a checkpoint
    if args.resume:
        if os.path.isfile(args.resume):
            print("=> loading checkpoint '{}'".format(args.resume))
            model.load_state_dict(torch.load(args.resume)['state_dict'])
            args.start_epoch = 0

        else:
            print("=> no checkpoint found at '{}'".format(args.resume))
    else:
        args.start_epoch = 0

    # Data loading code
    load_real_images = False
    train_dataset = freicar_segreg_dataloader.FreiCarLoader("../data/", padding=(0, 0, 12, 12),
                                       split='training', load_real=load_real_images)

    train_loader = torch.utils.data.DataLoader(train_dataset, batch_size=args.batch_size, shuffle=True, num_workers=4,
                                               pin_memory=False, drop_last=True)

    eval_dataset = freicar_segreg_dataloader.FreiCarLoader("../data/", padding=(0, 0, 12, 12),
                                                           split='validation', load_real=load_real_images)

    eval_loader = torch.utils.data.DataLoader(eval_dataset, batch_size=1, shuffle=True, num_workers=1,
                                              pin_memory=False, drop_last=False)

    # If --evaluate is passed from the command line --> evaluate
    if args.evaluate:

        eval(eval_loader, model)

    for epoch in range(args.start_epoch, num_epochs):
        # train for one epoch
        train(train_loader, model, optimizer, scheduler, epoch, eval_loader)

        # remember best iou and save checkpoint
        save_checkpoint({
            'epoch': epoch + 1,
            'state_dict': model.state_dict(),
            'best_iou': best_iou,
            'optimizer': optimizer.state_dict(),
        }, False)


def train(train_loader, model, optimizer, scheduler, epoch, eval_loader):
    batch_time = AverageMeter()
    data_time = AverageMeter()
    losses = AverageMeter()

    criterion_cross = torch.nn.CrossEntropyLoss()
    criterion_l1 = torch.nn.L1Loss()

    # switch to train mode
    model.train()

    IoU_all = []
    IoU_background_all = []
    IoU_road_all = []
    IoU_junction_all = []
    IoU_car_all = []

    miou = 0
    metric = SegmentationMetric(4)

    end = time.time()
    for i, (sample) in enumerate(train_loader):

        data_time.update(time.time() - end)

        image = sample['rgb'].cuda().float()
        lane_reg = sample['reg'].cuda().float()
        seg_ids = sample['seg'].cuda()

        ######################################
        # TODO: Implement me! Train Loop
        ######################################
        optimizer.zero_grad()

        # run forward
        image = torch.squeeze(image)

        seg_cl, seg_reg = model(image)

        seg_ids = torch.squeeze(seg_ids).long()
        lane_reg = torch.squeeze(lane_reg).long()

        # calculate loss
        loss_cl = criterion_cross(seg_cl, seg_ids)
        loss_reg = criterion_l1(seg_reg, lane_reg)

        loss = loss_cl + loss_reg * 0.03
        # print(loss)

        # backpropagation
        loss.backward()

        # optimize
        optimizer.step()

        # measure accuracy and record loss
        losses.update(loss.item(), image.size(0))

        # measure elapsed time
        batch_time.update(time.time() - end)
        end = time.time()

        pred = torch.argmax(seg_cl, 1)
        pred = pred.cpu().data.numpy()

        #mIoU calculation
        background = []
        road = []
        junction = []
        car = []
        background_seg = []
        road_seg = []
        junction_seg = []
        car_seg = []

        for a, b in np.nditer([pred, seg_ids.cpu().numpy()]):
            if (a == 0):
                background.append(a)
                background_seg.append(b)
            if (a == 1):
                road.append(a)
                road_seg.append(b)
            if (a == 2):
                junction.append(a)
                junction_seg.append(b)
            if (a == 3):
                car.append(a)
                car_seg.append(b)

        metric.update(np.array(background), np.array(background_seg))
        pixacc_background, IoU_background = metric.get()
        IoU_background_all.append(IoU_background)

        metric.update(np.array(road), np.array(road_seg))
        pixacc_road, IoU_road = metric.get()
        IoU_road_all.append(IoU_road)

        metric.update(np.array(junction), np.array(junction_seg))
        pixacc_junction, IoU_junction = metric.get()
        IoU_junction_all.append(IoU_junction)

        metric.update(np.array(car), np.array(car_seg))
        pixacc_car, IoU_car = metric.get()
        IoU_car_all.append(IoU_car)

        metric.update(pred, seg_ids.cpu().numpy())
        pixacc, IoU = metric.get()
        IoU_all.append(IoU)

        ## priting calculation ##
        # print("IoU")
        # print(IoU)
        # print("pixacc")
        # print(pixacc)
        # print(loss_reg.item())

        if i % args.print_freq == 0:
            print('Epoch: [{0}][{1}/{2}]\t'
                  'Time {batch_time.val:.3f} ({batch_time.avg:.3f})\t'
                  'Data {data_time.val:.3f} ({data_time.avg:.3f})\t'
                  'Loss {loss.val:.4f} ({loss.avg:.4f})'.format(
                epoch, i, len(train_loader), batch_time=batch_time,
                data_time=data_time, loss=losses))

    mIoU = np.mean(IoU_all)
    mIoU_background = np.mean(IoU_background_all)
    mIoU_junction = np.mean(IoU_junction_all)
    mIoU_road = np.mean(IoU_road_all)
    mIoU_car = np.mean(IoU_car_all)

    #print results
    print("loss")
    print(loss_reg.item())
    print("mean_iou_background")
    print(mIoU_background)
    print("mean_iou_junction")
    print(mIoU_junction)
    print("mean_iou_road")
    print(mIoU_road)
    print("mean_iou_car")
    print(mIoU_car)
    print("mean_iou")
    print(mIoU)
    scheduler.step()

    #logging the results
    with open('./logs/training_miou_background.txt', 'a') as f:
        f.write(str(mIoU_background) + ",")
        f.close()
    with open('./logs/training_miou_junction.txt', 'a') as f:
        f.write(str(mIoU_junction) + ",")
        f.close()
    with open('./logs/training_miou_road.txt', 'a') as f:
        f.write(str(mIoU_road) + ",")
        f.close()
    with open('./logs/training_miou_car.txt', 'a') as f:
        f.write(str(mIoU_car) + ",")
        f.close()
    with open('./logs/training_miou.txt', 'a') as f:
        f.write(str(mIoU) + ",")
        f.close()
    with open('./logs/training_L1.txt', 'a') as f:
        f.write(str(loss_reg.item()) + ",")
        f.close()

    if epoch % 1 == 0:
        eval(eval_loader, model)
    scheduler.step()

def eval(data_loader, model):

    metric = SegmentationMetric(4)
    metric.reset()

    model.eval()

    IoU_all = []
    IoU_background_all = []
    IoU_road_all = []
    IoU_junction_all = []
    IoU_car_all = []

    criterion_l1 = torch.nn.L1Loss()

    for i, (sample) in enumerate(data_loader):

        image = sample['rgb'].cuda().float()
        lane_reg = sample['reg'].cuda().float()
        seg_ids = sample['seg'].cuda()

        ######################################
        # TODO: Implement me!
        # You should calculate the IoU every N
        # epochs for both the training
        # and the evaluation set.
        # For segmentation color coding
        # see: color_coder.py
        ######################################

        seg, reg = model(image.squeeze(1))
        lane_reg = lane_reg.squeeze(0)
        loss_reg = criterion_l1(reg, lane_reg)
        seg_ids = seg_ids.squeeze(0)

        pred = torch.argmax(seg, 1)
        pred = pred.cpu().data.numpy()

        #mIoU calculation
        background = []
        road = []
        junction = []
        car = []
        background_seg = []
        road_seg = []
        junction_seg = []
        car_seg = []

        for a, b in np.nditer([pred, seg_ids.cpu().numpy()]):
            if (a == 0):
                background.append(a)
                background_seg.append(b)
            if (a == 1):
                road.append(a)
                road_seg.append(b)
            if (a == 2):
                junction.append(a)
                junction_seg.append(b)
            if (a == 3):
                car.append(a)
                car_seg.append(b)

        metric.update(np.array(background), np.array(background_seg))
        pixacc_background, IoU_background = metric.get()
        IoU_background_all.append(IoU_background)

        metric.update(np.array(road), np.array(road_seg))
        pixacc_road, IoU_road = metric.get()
        IoU_road_all.append(IoU_road)

        metric.update(np.array(junction), np.array(junction_seg))
        pixacc_junction, IoU_junction = metric.get()
        IoU_junction_all.append(IoU_junction)

        metric.update(np.array(car), np.array(car_seg))
        pixacc_car, IoU_car = metric.get()
        IoU_car_all.append(IoU_car)

        metric.update(pred, seg_ids[0].cpu().numpy())
        pixacc, IoU = metric.get()
        IoU_all.append(IoU)

        #print results
        # print("IoU")
        # print(IoU)
        # print("pixacc")
        # print(pixacc)
        # print("l1")
        # print(loss_reg.item())

    mIoU = np.mean(IoU_all)
    mIoU_background = np.mean(IoU_background_all)
    mIoU_junction = np.mean(IoU_junction_all)
    mIoU_road = np.mean(IoU_road_all)
    mIoU_car = np.mean(IoU_car_all)

    #logging the results
    with open("./logs/evaluation_miou_background.txt", "a") as c:
        c.write(str(mIoU_background) + ",")
        c.close()
    with open("./logs/evaluation_miou_junction.txt", "a") as c:
        c.write(str(mIoU_junction) + ",")
        c.close()
    with open("./logs/evaluation_miou_road.txt", "a") as c:
        c.write(str(mIoU_road) + ",")
        c.close()
    with open("./logs/evaluation_miou_car.txt", "a") as c:
        c.write(str(mIoU_car) + ",")
        c.close()
    with open("./logs/evaluation_miou.txt", "a") as c:
        c.write(str(mIoU) + ",")
        c.close()
    with open('./logs/evaluation_L1.txt', 'a') as f:
        f.write(str(loss_reg.item()) + ",")
        f.close()

def save_checkpoint(state, is_best, filename='checkpoint.pth.tar'):
    torch.save(state, filename)
    if is_best:
        shutil.copyfile(filename, 'model_best.pth.tar')

class AverageMeter(object):
    """Computes and stores the average and current value"""

    def __init__(self):
        self.reset()

    def reset(self):
        self.val = 0
        self.avg = 0
        self.sum = 0
        self.count = 0

    def update(self, val, n=1):
        self.val = val
        self.sum += val * n
        self.count += n
        self.avg = self.sum / self.count

if __name__ == '__main__':
    main()