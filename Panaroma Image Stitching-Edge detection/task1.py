"""
Image Stitching Problem
(Due date: Nov. 26, 11:59 P.M., 2021)

The goal of this task is to stitch two images of overlap into one image.
You are given 'left.jpg' and 'right.jpg' for your image stitching code testing. 
Note that different left/right images might be used when grading your code. 

To this end, you need to find keypoints (points of interest) in the given left and right images.
Then, use proper feature descriptors to extract features for these keypoints. 
Next, you should match the keypoints in both images using the feature distance via KNN (k=2); 
cross-checking and ratio test might be helpful for feature matching. 
After this, you need to implement RANSAC algorithm to estimate homography matrix. 
(If you want to make your result reproducible, you can try and set fixed random seed)
At last, you can make a panorama, warp one image and stitch it to another one using the homography transform.
Note that your final panorama image should NOT be cropped or missing any region of left/right image. 

Do NOT modify the code provided to you.
You are allowed use APIs provided by numpy and opencv, except “cv2.findHomography()” and
APIs that have “stitch”, “Stitch”, “match” or “Match” in their names, e.g., “cv2.BFMatcher()” and
“cv2.Stitcher.create()”.
If you intend to use SIFT feature, make sure your OpenCV version is 3.4.2.17, see project2.pdf for details.
"""

import cv2
import numpy as np
# np.random.seed(<int>) # you can use this line to set the fixed random seed if you are using np.random
import random

from numpy.core.numeric import Inf
# random.seed(<int>) # you can use this line to set the fixed random seed if you are using random


def solution(left_img, right_img):
    """
    :param left_img:
    :param right_img:
    :return: you need to return the result panorama image which is stitched by left_img and right_img
    """
    gray_left= cv2.cvtColor(left_img,cv2.COLOR_BGR2GRAY)
    gray_right= cv2.cvtColor(right_img,cv2.COLOR_BGR2GRAY)

    # TO DO: implement your solution here
    sift = cv2.xfeatures2d.SIFT_create()
    kp1, des1 = sift.detectAndCompute(gray_left, None)
    kp2, des2 = sift.detectAndCompute(gray_right, None)
    
    ardes1 = np.asarray(des1)
    ardes2 = np.asarray(des2)
    print(len(ardes2),len(ardes1))
    match_12 = np.empty(shape=(0,2))
    for i in range(0,len(ardes1)):
        dist = []
        dist = np.sqrt(np.sum((ardes1[i]-ardes2)**2,axis = 1))
        rank = np.argsort(dist)
        index1 = rank[0]#rank.tolist().index(0)
        index2 = rank[1]#rank.tolist().index(1)
        if(dist[index1] < 0.75 * dist[index2]):
            match_12 = np.append(match_12, [[i,index1]], axis=0)

    #print(len(match_12))
    x1 = []#np.empty(shape=(0,1))
    y1 = []#np.empty(shape=(0,1))
    x2 = []#np.empty(shape=(0,1))
    y2 = []#np.empty(shape=(0,1))   

    for j in match_12:
        x1.append(kp1[int(j[0])].pt[0])#np.append(x1,[[kp1[int(j[0])].pt[0]]],axis=0)
        y1.append(kp1[int(j[0])].pt[1])#= np.append(y1,[[kp1[int(j[0])].pt[1]]],axis=0)
        x2.append(kp2[int(j[1])].pt[0])#= np.append(x2,[[kp2[int(j[1])].pt[0]]],axis=0)
        y2.append(kp2[int(j[1])].pt[1])#= np.append(y2,[[kp2[int(j[1])].pt[1]]],axis=0)
    
    #RANSAC
    t = 5
    inline = 0
    for iter in range(0,7000):
        
        xl = x1.copy()
        yl = y1.copy()
        xr = x2.copy()
        yr = y2.copy()

        x3,y3,x4,y4 =[],[],[],[]
        list = []
        list = random.sample(range(0,len(xl)), 4)
        #print(list)
        for l in sorted(list, reverse=True):
            #print(xl[176])
            x3.append(xl[l])
            xl.remove(xl[l])

            y3.append(yl[l])
            yl.remove(yl[l])

            x4.append(xr[l])
            xr.remove(xr[l])

            y4.append(yr[l])
            yr.remove(yr[l])
        


        calibMatrix2 = [ [0]*9 for _ in range(2*len(x3)) ]
        index = 0 
        for i in range(0,2*len(x3),2):
                calibMatrix2[i][0]=x3[index]
                calibMatrix2[i][1]=y3[index]
                calibMatrix2[i][2]=1
                
                calibMatrix2[i][6]= -(x4[index]*x3[index])
                calibMatrix2[i][7]= -(x4[index]*y3[index])
                calibMatrix2[i][8]= -(x4[index])
                
                calibMatrix2[i+1][3]=x3[index]
                calibMatrix2[i+1][4]=y3[index]
                calibMatrix2[i+1][5]=1
                
                calibMatrix2[i+1][6]= -(y4[index]*x3[index])
                calibMatrix2[i+1][7]= -(y4[index]*y3[index])
                calibMatrix2[i+1][8]= -(y4[index])
                
                index += 1

        u2,sigma2,vt = np.linalg.svd(calibMatrix2)  
        #print(vt)  
        h = vt[8]
        h22 = h[8]
        h = [j/h22 for j in h]
        inl = 0
        #print(h)
        for i in range(0,len(xl)):
            xp = (h[0]*xl[i]+h[1]*yl[i]+h[2])/(h[6]*xl[i]+h[7]*yl[i]+h[8])
            yp = (h[3]*xl[i]+h[4]*yl[i]+h[5])/(h[6]*xl[i]+h[7]*yl[i]+h[8])
            test = np.sqrt((xp-xr[i])**2 + (yp-yr[i])**2)
            if test < t:
                inl += 1
        if inl > inline:
            inline = inl
            h_f = np.reshape(h,(3,3))
        
        
    print(inline,h_f)
    #cv2.imshow('showkey',)
    #result_img = cv2.drawKeypoints(left_img,kp1,None)

    """ outer_x= left_img.shape[1]+right_img.shape[1]
    print(left_img.shape,right_img.shape)
    outer_y= left_img.shape[0]*1
    
    off_arr = np.asarray([[1,0,607],[0,1,25],[0,0,1]])
    h_final = np.matmul(off_arr,h_f)
    a = np.array([[0, 0], [outer_x, 0], [0,outer_y], [outer_x,outer_y]], dtype='float32')
    h = np.array(h_f, dtype='float32')
    a = np.array([a])
    pointsOut = cv2.perspectiveTransform(a, h_f)
    print(pointsOut)
    result_img = cv2.warpPerspective(left_img,h_f, (outer_x, outer_y)) """
    h1,w1 = left_img.shape[:2]
    h2,w2 = right_img.shape[:2]
    pts1 = np.asarray([[0,0],[0,h1],[w1,h1],[w1,0]],dtype='float32').reshape(-1,1,2)
    pts2 = np.asarray([[0,0],[0,h2],[w2,h2],[w2,0]],dtype='float32').reshape(-1,1,2)
    pts2_ = cv2.perspectiveTransform(pts1, h_f)
    pts = np.concatenate((pts2_,pts2), axis=0)
    [xmin, ymin] = (np.ravel(np.min(pts,axis=0)))
    [xmax, ymax] = (np.ravel(np.max(pts,axis=0)))
    t = [-int(xmin),-int(ymin)]
    Ht = np.asarray([[1,0,t[0]],[0,1,t[1]],[0,0,1]]) # translate

    result = cv2.warpPerspective(left_img, Ht.dot(h_f), (int(xmax)-int(xmin), int(ymax)-int(ymin)))
    result[t[1]:h2+t[1],t[0]:w2+t[0]] = right_img
    #result_img = cv2.perspectiveTransform(left_img,h_f,(outer_x, outer_y))
    return result
    raise NotImplementedError
    
    

if __name__ == "__main__":
    left_img = cv2.imread('left.jpg')
    right_img = cv2.imread('right.jpg')
    result_img = solution(left_img, right_img)
    cv2.imwrite('results/task1_result.jpg', result_img)


