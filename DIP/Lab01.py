# -*- coding: utf-8 -*-
"""
Created on Fri Feb  4 09:18:54 2022

@author: jonas
"""

"""
Exercise 1 (imports)
"""

from imageio import imread, imwrite # Library for reading and writing images
from pylab import * # library containing lots of basic functions for data processing
from skimage.util import img_as_ubyte, img_as_float # Convert image between interval ~(0,255) and ~(0,1)
import numpy as np # Library for handling matrices
import matplotlib.pyplot as plt # Library for creating plots
from matplotlib.pyplot import imshow # import imshow directly since we will use it a lot
plt.gray() #specify gray-scale colormap for plots

"""
Exercise 2
"""

im = imread('Images/cameraman.tif')

print('Min pixel value: ', np.min(im))
print('Max pixel value: ', im.max())

"""
Exercise 3
"""

plt.figure()
imshow(im)


"""
Exercise 4
"""

plt.figure()
imshow(im[0:-1, 49:99])

plt.figure()
imshow(im[0:-1, 99:149])

"""
Exercise 5
"""

plt.figure()
plt.hist(im.flatten(), bins=50)

"""
Exercise 6
"""

new_im = np.zeros((len(im[0]), len(im)))
for i in range(len(im[0]) - 1):
    for k in range(len(im) - 1):
        if (im[i][k] > im.mean()):
            new_im[i][k] = uint8(255)
        else:
            new_im[i][k] = uint8(0)
      
plt.figure()
imshow(new_im)
