

import numpy as np
import cv2
import matplotlib.pyplot as plt


def showMarker(im):
    plt.imshow(im, interpolation='nearest', cmap='gray')

def read_image(fname):
    im = cv2.imread(fname)
    im = cv2.cvtColor(im, cv2.COLOR_BGR2RGB)
    return im