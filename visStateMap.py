import numpy as np
from cv2 import cv2
import time
import os
import torch
import torchvision


def readYML(ymlFile):
    fs = cv2.FileStorage(ymlFile,cv2.FileStorage_READ)
    if not fs.isOpened():
        print('Cannot open yml file, program exit')
        exit(-2)
    data = []
    simulationTime = fs.getNode('simulationTime').real()/60 # minutes
    toRight = fs.getNode('toRight').mat()/simulationTime
    toLeft = fs.getNode('toLeft').mat()/simulationTime
    toUp = fs.getNode('toUp').mat()/simulationTime
    toDown = fs.getNode('toDown').mat()/simulationTime
    
    toRight = cv2.resize(toRight,(512,512))
    toLeft = cv2.resize(toLeft,(512,512))
    toUp = cv2.resize(toUp,(512,512))
    toDown = cv2.resize(toDown,(512,512))

    data = np.concatenate((toLeft[np.newaxis,:],toRight[np.newaxis,:],toUp[np.newaxis,:],toDown[np.newaxis,:]),axis=0)
    fs.release()
    return data

def readNpy(npyFile):
    data = np.load(npyFile)
    return data


def splitVis(datas):
    # 分别可视化每个通道
    dim = len(np.shape(datas))
    def processSingleImg(data):
        toLeft = data[0]
        toRight = data[1]
        toUp = data[2]
        toDown = data[3]
        peopleNum = toLeft + toRight + toUp + toDown
        size = np.shape(toLeft)[0]

        peopleNumFigure = np.zeros([size,size,3])
        toLeftFigure = np.zeros([size,size,3])
        toRightFigure = np.zeros([size,size,3])
        toUpFigure = np.zeros([size,size,3])
        toDownFigure = np.zeros([size,size,3])

        visThreshold = 70
        singleVisThreshold = visThreshold / 2

        peopleNum[peopleNum > visThreshold] = visThreshold
        peopleNum = peopleNum * 255/visThreshold
        toLeft[toLeft > singleVisThreshold] = singleVisThreshold
        toLeft = toLeft * 255 / singleVisThreshold
        toRight[toRight > singleVisThreshold] = singleVisThreshold
        toRight = toRight * 255 / singleVisThreshold
        toUp[toUp > singleVisThreshold] = singleVisThreshold
        toUp = toUp * 255 / singleVisThreshold
        toDown[toDown > singleVisThreshold] = singleVisThreshold
        toDown = toDown * 255 /singleVisThreshold

        peopleNumFigure[:,:,0] = 255 - peopleNum
        peopleNumFigure[:,:,1] = 255 - peopleNum
        peopleNumFigure[:,:,2] = 255 - peopleNum
        
        toLeftFigure[:,:,0] = 255 - toLeft
        toLeftFigure[:,:,1] = 255 - toLeft
        toLeftFigure[:,:,2] = 255

        toRightFigure[:,:,0] = 255
        toRightFigure[:,:,1] = 255 - toRight
        toRightFigure[:,:,2] = 255 - toRight

        toUpFigure[:,:,0] = 255 - toUp
        toUpFigure[:,:,1] = ((255 - 92) * (255 - toUp) / 255)  + 92
        toUpFigure[:,:,2] = 255

        toDownFigure[:,:,0] = 255
        toDownFigure[:,:,1] = 255 - toDown
        toDownFigure[:,:,2] = ((255 - 158) * (255 - toDown) / 255)  + 158

        peopleNumFigure = peopleNumFigure.astype(np.uint8)
        toLeftFigure = toLeftFigure.astype(np.uint8)
        toRightFigure = toRightFigure.astype(np.uint8)
        toUpFigure = toUpFigure.astype(np.uint8)
        toDownFigure = toDownFigure.astype(np.uint8)

        x0 = 50
        y0 = 50
        arrowLenth = 50
        cv2.arrowedLine(toRightFigure,(x0 - int(arrowLenth / 2), y0),(x0 + int(arrowLenth / 2), y0),(255, 0, 0), 2)
        cv2.arrowedLine(toLeftFigure,(x0 + int(arrowLenth / 2), y0),(x0 - int(arrowLenth / 2), y0),(0, 0, 255), 2)
        cv2.arrowedLine(toDownFigure,(x0, y0 - int(arrowLenth / 2)),(x0, y0 + int(arrowLenth / 2)),(255, 0, 158), 2)
        cv2.arrowedLine(toUpFigure,(x0, y0 + int(arrowLenth / 2)),(x0, y0 - int(arrowLenth / 2)),(0, 92, 255), 2)

        bgr = np.zeros([size,size*5,3],dtype = np.uint8)
        bgr[:,size*0:size*1,:] = toUpFigure
        bgr[:,size*1:size*2,:] = toDownFigure
        bgr[:,size*2:size*3,:] = peopleNumFigure
        bgr[:,size*3:size*4,:] = toLeftFigure
        bgr[:,size*4:size*5,:] = toRightFigure
        return bgr

    if dim == 3: 
        bgr = processSingleImg(datas)
        return bgr
    
    if dim == 4:
        # datas = datas.numpy()
        bgrs = []
        for k in range(np.shape(datas)[0]):
            bgr = processSingleImg(datas[k])
            bgrs.append(bgr)
        bgrs = np.array(bgrs)
        bgrs = np.transpose(bgrs,(0,3,1,2))
        bgrs = torch.Tensor(bgrs)
        bgrs = torchvision.utils.make_grid(bgrs,nrow=4,normalize=False,pad_value=0)
        bgrs = bgrs.numpy()
        bgrs = bgrs.astype(np.uint8)
        bgrs = np.transpose(bgrs,(1,2,0))
        return bgrs

    else:
        print('dim error! check data dim.(dim should be 3 or 4)')
        exit(-1)


def mergedVis(datas):
    # 把所有通道可视化到一张图上
    dim = len(np.shape(datas))
    def processSingleImg(data):
        toLeft = np.copy(data[0])
        toRight = np.copy(data[1])
        toUp = np.copy(data[2])
        toDown = np.copy(data[3])
        peopleNum = toLeft + toRight + toUp + toDown
        size = np.shape(toLeft)[0]

        singleVisThreshold = 25

        toLeft[toLeft > singleVisThreshold] = singleVisThreshold
        toLeft = toLeft * 255 / singleVisThreshold
        toRight[toRight > singleVisThreshold] = singleVisThreshold
        toRight = toRight * 255 / singleVisThreshold
        toUp[toUp > singleVisThreshold] = singleVisThreshold
        toUp = toUp * 255 / singleVisThreshold
        toDown[toDown > singleVisThreshold] = singleVisThreshold
        toDown = toDown * 255 /singleVisThreshold

        bgr = np.ones([size,size,3],dtype = np.uint8)
        maxValue = np.max(data,axis=0)
        bgr[maxValue==data[0],0] = 255 - (255-0)*toLeft[maxValue==data[0]]/255
        bgr[maxValue==data[0],1] = 255 - (255-0)*toLeft[maxValue==data[0]]/255
        bgr[maxValue==data[0],2] = 255 - (255-255)*toLeft[maxValue==data[0]]/255

        bgr[maxValue==data[1],0] = 255 - (255-255)*toRight[maxValue==data[1]]/255
        bgr[maxValue==data[1],1] = 255 - (255-0)*toRight[maxValue==data[1]]/255
        bgr[maxValue==data[1],2] = 255 - (255-0)*toRight[maxValue==data[1]]/255

        bgr[maxValue==data[2],0] = 255 - (255-102)*toUp[maxValue==data[2]]/255
        bgr[maxValue==data[2],1] = 255 - (255-207)*toUp[maxValue==data[2]]/255    
        bgr[maxValue==data[2],2] = 255 - (255-49)*toUp[maxValue==data[2]]/255

        bgr[maxValue==data[3],0] = 255 - (255-255)*toDown[maxValue==data[3]]/255
        bgr[maxValue==data[3],1] = 255 - (255-0)*toDown[maxValue==data[3]]/255
        bgr[maxValue==data[3],2] = 255 - (255-158)*toDown[maxValue==data[3]]/255    

        return bgr

    if dim == 3: 
        bgr = processSingleImg(datas)
        return bgr
    
    if dim == 4:
        # datas = datas.numpy()
        bgrs = []
        for k in range(np.shape(datas)[0]):
            bgr = processSingleImg(datas[k])
            bgrs.append(bgr)
        bgrs = np.array(bgrs)
        bgrs = np.transpose(bgrs,(0,3,1,2))
        bgrs = torch.Tensor(bgrs)
        bgrs = torchvision.utils.make_grid(bgrs,nrow=4,normalize=False,pad_value=0)
        bgrs = bgrs.numpy()
        bgrs = bgrs.astype(np.uint8)
        bgrs = np.transpose(bgrs,(1,2,0))
        return bgrs

    else:
        print('dim error! check data dim.(dim should be 3 or 4)')
        exit(-1)


# 这个函数没啥用了
# def visAsOneImg(data):
#     #以黑色为底色
#     toLeft = np.copy(data[0])
#     toRight = np.copy(data[1])
#     toUp = np.copy(data[2])
#     toDown = np.copy(data[3])
#     peopleNum = toLeft + toRight + toUp + toDown
#     size = np.shape(toLeft)[0]

#     visThreshold = 70
#     singleVisThreshold = visThreshold / 2

#     peopleNum[peopleNum > visThreshold] = visThreshold
#     peopleNum = peopleNum * 255/visThreshold
#     toLeft[toLeft > singleVisThreshold] = singleVisThreshold
#     toLeft = toLeft * 255 / singleVisThreshold
#     toRight[toRight > singleVisThreshold] = singleVisThreshold
#     toRight = toRight * 255 / singleVisThreshold
#     toUp[toUp > singleVisThreshold] = singleVisThreshold
#     toUp = toUp * 255 / singleVisThreshold
#     toDown[toDown > singleVisThreshold] = singleVisThreshold
#     toDown = toDown * 255 /singleVisThreshold

#     bgr = np.ones([size,size,3],dtype = np.uint8)
#     maxValue = np.max(data,axis=0)
#     bgr[maxValue==data[0],0] = 0
#     bgr[maxValue==data[0],1] = 0
#     bgr[maxValue==data[0],2] = 255 - toLeft[maxValue==data[0]]

#     bgr[maxValue==data[1],0] = toRight[maxValue==data[1]]
#     bgr[maxValue==data[1],1] = 0
#     bgr[maxValue==data[1],2] = 0

#     bgr[maxValue==data[2],0] = 0
#     bgr[maxValue==data[2],1] = 92 * toUp[maxValue==data[2]]/255
#     bgr[maxValue==data[2],2] = toUp[maxValue==data[2]]

#     bgr[maxValue==data[3],0] = toDown[maxValue==data[3]]
#     bgr[maxValue==data[3],1] = 0
#     bgr[maxValue==data[3],2] = 158 * toDown[maxValue==data[3]]/255
#     return bgr



########################### 下面这一块是把网络里保存的npy文件可视化出来
# ymlFile = 'D:/Research/IV2020MapPrediction/Code/MapPrediction/GenerateFakeMap/datas/EastSouthGate/stateMapDir5/EastSouth_M6_P0.yml'
# data = readYML(ymlFile)
# npyFile = 'D:/Research/IV2020MapPrediction/Code/MapPrediction/GenerateFakeMap/python/Test_Epoch3.npy'
# data = readNpy(npyFile)

# vis = mergedVis(data)
# resizeCoeffi = 0.4
# vis = cv2.resize(vis,(int(np.shape(vis)[1]*resizeCoeffi),int(np.shape(vis)[0]*resizeCoeffi)))
# cv2.imshow('img',vis)
# cv2.waitKey(0)
# cv2.imwrite('test.png',vis)


########################### 下面这一块是把原始数据文件夹下的所有yml文件保存成可视化的图像
for i in range(1,721):
    ymlFile = 'D:/Research/IV2020MapPrediction/Code/MapPrediction/GenerateFakeMap/datas/EastGate/stateMapDir7/East_M%d_P0.yml'%i
    print(i)
    data = readYML(ymlFile)
    start = time.time()
    vis = splitVis(data)
    end = time.time()
    # print(end-start)
    start = time.time()
    vis = mergedVis(data)
    end = time.time()
    # print(end-start)
    cv2.imwrite('D:/Research/IV2020MapPrediction/Code/MapPrediction/GenerateFakeMap/datas/EastGate/stateMapDir7/vis2/East_M%d_P0.png'%i,vis)

