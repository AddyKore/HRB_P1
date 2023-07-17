#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Jul 16 20:45:49 2023

@author: hrb_xubu
"""
from joy import JoyApp, progress
import numpy as np
from numpy.random import randn
n=np.zeros([3,1])
n1=np.array([[3],[3],[3]])
progress(n+n1)