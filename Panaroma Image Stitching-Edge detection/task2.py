"""
 Grayscale Image Processing
(Due date: Nov. 26, 11:59 P.M., 2021)

The goal of this task is to experiment with two commonly used 
image processing techniques: image denoising and edge detection. 
Specifically, you are given a grayscale image with salt-and-pepper noise, 
which is named 'task2.png' for your code testing. 
Note that different image might be used when grading your code. 

You are required to write programs to: 
(i) denoise the image using 3x3 median filter;
(ii) detect edges in the denoised image along both x and y directions using Sobel operators (provided in line 30-32).
(iii) design two 3x3 kernels and detect edges in the denoised image along both 45° and 135° diagonal directions.
Hint: 
• Zero-padding is needed before filtering or convolution. 
• Normalization is needed before saving edge images. You can normalize image using the following equation:
    normalized_img = 255 * frac{img - min(img)}{max(img) - min(img)}

Do NOT modify the code provided to you.
You are NOT allowed to use OpenCV library except the functions we already been imported from cv2. 
You are allowed to use Numpy for basic matrix calculations EXCEPT any function/operation related to convolution or correlation. 
You should NOT use any other libraries, which provide APIs for convolution/correlation ormedian filtering. 
Please write the convolution code ON YOUR OWN. 
"""

from cv2 import imread, imwrite, imshow, IMREAD_GRAYSCALE, namedWindow, waitKey, destroyAllWindows
import numpy as np
from numpy.core.fromnumeric import size
from numpy.lib.shape_base import expand_dims

# Sobel operators are given here, do NOT modify them.
sobel_x = np.array([[1, 0, -1], [2, 0, -2], [1, 0, -1]]).astype(int)
sobel_y = np.array([[1, 2, 1], [0, 0, 0], [-1, -2, -1]]).astype(int)


def filter(img):
    """
    :param img: numpy.ndarray(int), image
    :return denoise_img: numpy.ndarray(int), image, same size as the input image

    Apply 3x3 Median Filter and reduce salt-and-pepper noises in the input noise image
    """

    # TO DO: implement your solution here
    size_img = img.shape
    p = np.zeros((size_img[0]+2,size_img[1]+2))
    
    for i in range(1,size_img[0]+1):
        for j in range(1,size_img[1]+1):
            p[i][j] = img[i-1][j-1]
    
    for i in range (1,size_img[0]+1):
        for j in range (1,size_img[1]+1):
            sum = []
            for k in range(-1,2):
                for l in range(-1,2):
                    sum.append(p[i+k,j+l])
            #print(sum)
            for m in range (1,len(sum)):
                current = sum[m]
                prev = m-1
                while prev >=0 and sum[prev]>current:
                    sum[prev+1] = sum[prev]
                    prev = prev - 1
                sum[prev+1] = current
            'print(sum)'
            p[i,j] = sum[4]

    denoise_img = np.zeros((size_img[0],size_img[1]))
    for i in range(0,size_img[0]):
        for j in range(0,size_img[1]):
            denoise_img[i,j] = p[i+1,j+1]
    
    return denoise_img        
    raise NotImplementedError
    


def convolve2d(img, kernel):
    """
    :param img: numpy.ndarray, image
    :param kernel: numpy.ndarray, kernel
    :return conv_img: numpy.ndarray, image, same size as the input image

    Convolves a given image (or matrix) and a given kernel.
    """

    # TO DO: implement your solution here
    
    p = np.zeros((img.shape[0]+2,img.shape[1]+2))
    g = np.zeros((img.shape[0],img.shape[1]))
    'print(kernel)'
    for i in range(1,img.shape[0]+1):
        for j in range(1,img.shape[1]+1):
            p[i,j] = img[i-1][j-1]
    
    for i in range(1,img.shape[0]+1):
        for j in range(1,img.shape[1]+1):
            a = 0
            for k in range(-1,2):
                for l in range(-1,2):
                    #print(a,k,l)
                    a = a + p[i+k][j+l]*kernel[1-k][1-l]
            g[i-1][j-1] = a
    #imshow('conv',p)
    
    return g
    raise NotImplementedError

def edge_detect(img):
    """
    :param img: numpy.ndarray(int), image
    :return edge_x: numpy.ndarray(int), image, same size as the input image, edges along x direction
    :return edge_y: numpy.ndarray(int), image, same size as the input image, edges along y direction
    :return edge_mag: numpy.ndarray(int), image, same size as the input image, 
                      magnitude of edges by combining edges along two orthogonal directions.

    Detect edges using Sobel kernel along x and y directions.
    Please use the Sobel operators provided in line 30-32.
    Calculate magnitude of edges by combining edges along two orthogonal directions.
    All returned images should be normalized to [0, 255].
    """

    # TO DO: implement your solution here
    edge_x = np.zeros((img.shape[0],img.shape[1]))
    edge_y = np.zeros((img.shape[0],img.shape[1]))
    edge_mag = np.zeros((img.shape[0],img.shape[1]))
    min = np.amin(img)
    max = np.amax(img)

    for i in range(0,img.shape[0]):
        for j in range(0,img.shape[1]):
            img[i,j] = np.double((img[i,j]-min)/(max-min))
            #print(img[i,j])
            
    
    edge_x = convolve2d(img,sobel_x)
    edge_y = convolve2d(img,sobel_y)
    for i in range(0,img.shape[0]):
        for j in range(0,img.shape[1]):
            edge_mag[i,j] = (edge_x[i,j]**2 + edge_y[i,j]**2)**0.5

    xmin = np.amin(edge_x)
    ymin = np.amin(edge_y)
    xmax = np.amax(edge_x)
    ymax = np.amax(edge_y)
    mmin = np.amin(edge_mag)
    mmax = np.amax(edge_mag)
    for i in range(0,img.shape[0]):
        for j in range(0,img.shape[1]):
            edge_x[i,j] = (int(255*((edge_x[i,j]-xmin)/(xmax-xmin))))
            edge_y[i,j] = (int(255*((edge_y[i,j]-ymin)/(ymax-ymin))))
            edge_mag[i,j] = (int(255*((edge_mag[i,j]-mmin)/(mmax-mmin))))
            #if (int(255*((edge_x[i,j]-xmin)/(xmax-xmin)))) > 125:
            #    edge_x[i,j] = 255
            #else:
            #    edge_x[i,j] = 0
            #if (int(255*((edge_y[i,j]-ymin)/(ymax-ymin)))) > 130:
            #    edge_y[i,j] = 255
            #else:
            #    edge_y[i,j] = 0
            #if (int(255*((edge_mag[i,j]-mmin)/(mmax-mmin)))) > 50:
            #    edge_mag[i,j] = 255
            #else:
            #    edge_mag[i,j] = 0

    return edge_x, edge_y ,edge_mag
    raise NotImplementedError

def edge_diag(img):
    """
    :param img: numpy.ndarray(int), image
    :return edge_45: numpy.ndarray(int), image, same size as the input image, edges along x direction
    :return edge_135: numpy.ndarray(int), image, same size as the input image, edges along y direction

    Design two 3x3 kernels to detect the diagonal edges of input image. Please print out the kernels you designed.
    Detect diagonal edges along 45° and 135° diagonal directions using the kernels you designed.
    All returned images should be normalized to [0, 255].
    """

    # TO DO: implement your solution here

    filter_45 = np.array([[0, -1, -2], [1, 0, -1], [2, 1, 0]]).astype(int)
    filter_135 = np.array([[-2, -1, 0], [-1, 0, 1], [0, 1, 2]]).astype(int)

    edge_45 = np.zeros((img.shape[0],img.shape[1]))
    edge_135 = np.zeros((img.shape[0],img.shape[1]))
    
    min = np.amin(img)
    max = np.amax(img)

    for i in range(0,img.shape[0]):
        for j in range(0,img.shape[1]):
            img[i,j] = np.double((img[i,j]-min)/(max-min))
            #print(img[i,j])
            
    
    edge_45 = convolve2d(img,filter_45)
    edge_135 = convolve2d(img,filter_135)
    

    xmin = np.amin(edge_45)
    ymin = np.amin(edge_135)
    xmax = np.amax(edge_45)
    ymax = np.amax(edge_135)
    
    for i in range(0,img.shape[0]):
        for j in range(0,img.shape[1]):
            edge_45[i,j] = (int(255*((edge_45[i,j]-xmin)/(xmax-xmin))))
            edge_135[i,j] = (int(255*((edge_135[i,j]-ymin)/(ymax-ymin))))
            #if(int(255*((edge_45[i,j]-xmin)/(xmax-xmin))))>155:
            #    edge_45[i,j] = 255
            #else:
            #    edge_45[i,j] = 0
            #if (int(255*((edge_135[i,j]-ymin)/(ymax-ymin)))) > 165:
            #    edge_135[i,j] = 255
            #else: 
            #    edge_135[i,j] = 0
    
    print(filter_45,"/n", filter_135) # print the two kernels you designed here
    return edge_45, edge_135
    raise NotImplementedError

if __name__ == "__main__":
    noise_img = imread('task2.png', IMREAD_GRAYSCALE)
    noise_img.shape
    'print(noise_img)'
    denoise_img = filter(noise_img)
    imwrite('results/task2_denoise.jpg', denoise_img)
    
    edge_x_img, edge_y_img, edge_mag_img = edge_detect(denoise_img)
     
    imwrite('results/task2_edge_x.jpg', edge_x_img)
    imwrite('results/task2_edge_y.jpg', edge_y_img)
    imwrite('results/task2_edge_mag.jpg', edge_mag_img)
    edge_45_img, edge_135_img = edge_diag(denoise_img)
    imwrite('results/task2_edge_diag1.jpg', edge_45_img)
    imwrite('results/task2_edge_diag2.jpg', edge_135_img)





