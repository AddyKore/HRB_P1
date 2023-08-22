#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Jul 17 14:18:08 2023

@author: hrb_xubu
"""
from numpy.random import randn
from numpy import c_, exp
from matplotlib.pyplot import *
xyt = randn(3,100)
z = c_[[1,1j,0] @ xyt,[1,1j,0] @ xyt + 0.1*exp(1j*xyt[-1,:])]
plot( z.T.real, z.T.imag, '-' )
plot(z[:,0].real,z[:,0].imag,'.')