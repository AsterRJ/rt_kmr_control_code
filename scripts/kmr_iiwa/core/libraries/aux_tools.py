#!/usr/bin/env python3

"""
      @author   Carlos Suarez Zapico carlossuarezzapico@gmail.com
 
      @internal
      Created  10-Oct-2022
      Company  Edinburgh Centre for Robotics
      Auxiliary tools and methods 
"""


# Standard imports
import numpy as np


class aux_tools():
    
    def VAL_SAT(self, value, maxValue, minValue):
        if (value >= maxValue):
            return maxValue
        elif (value <= minValue):
            return minValue
        else:
            return value
    
    def VAL_SAT_np_array(self, array, maxValue, minValue):
        return  np.clip(array, minValue, maxValue)
    
    def skew(self, x):
        return np.array([[0, -x[2], x[1]],
                     [x[2], 0, -x[0]],
                     [-x[1], x[0], 0]])
        
    def fill_static_array(self, np_array, new_value):
        """
        Example on how the function works:
        [1, 2, 3, 4, 5] inserting a new component 10 results in [10, 1, 2, 3, 4]
        """
        aux = np.empty_like(np_array)
        aux[:1] = new_value
        aux[1:] = np_array[:-1]
        return aux
    
    def kdl_to_mat(self, data):
        mat = np.mat(np.zeros((data.rows(), data.columns())))
        for i in range(data.rows()):
            for j in range(data.columns()):
                mat[i, j] = data[i, j]
        return mat
    
    def copy_npArray_to_sharedArray(self, npArray, sharedArray):
        for i in range(len(sharedArray)):
            sharedArray[i] = npArray[i]    

    def copy_sharedArray_to_npArray(self, sharedArray, npArray):
        for i in range(len(npArray)):
            npArray[i] = sharedArray[i]
    
    def convert_Transform_shared_to_npmatrix(self, sharedArray):
        T = np.matrix([[sharedArray[0], sharedArray[1], sharedArray[2], sharedArray[3]], 
                       [sharedArray[4], sharedArray[5], sharedArray[6], sharedArray[7]], 
                       [sharedArray[8], sharedArray[9], sharedArray[10], sharedArray[11]],
                       [sharedArray[12], sharedArray[13], sharedArray[14], sharedArray[15]]]) 
        return T
    
    def convert_2DTransform_shared_to_2Dnpmatrix(self, sharedArray):
        T = np.matrix([[sharedArray[0], sharedArray[1], sharedArray[2]], 
                       [sharedArray[3], sharedArray[4], sharedArray[5]], 
                       [sharedArray[6], sharedArray[7], sharedArray[8]]]) 
        return T
    
    
    def convert_3DTransform_shared_to_2Dnpmatrix(self, sharedArray):
        T = np.matrix([[sharedArray[0], sharedArray[1], sharedArray[3]], 
                       [sharedArray[4], sharedArray[5], sharedArray[7]], 
                       [0.0,            0.0,            1.0]]) 
        return T
    
    def convert_Transform_npmatrix_to_shared(self, Tmatrix, sharedArray):
        sharedArray[0] = Tmatrix[0,0]; sharedArray[1] = Tmatrix[0,1]; sharedArray[2] = Tmatrix[0,2]; sharedArray[3] = Tmatrix[0,3]
        sharedArray[4] = Tmatrix[1,0]; sharedArray[5] = Tmatrix[1,1]; sharedArray[6] = Tmatrix[1,2]; sharedArray[7] = Tmatrix[1,3]
        sharedArray[8] = Tmatrix[2,0]; sharedArray[9] = Tmatrix[2,1]; sharedArray[10] = Tmatrix[2,2]; sharedArray[11] = Tmatrix[2,3]
        sharedArray[12] = Tmatrix[3,0]; sharedArray[13] = Tmatrix[3,1]; sharedArray[14] = Tmatrix[3,2]; sharedArray[15] = Tmatrix[3,3]
    
    def convert_2DTransform_npmatrix_to_2Dshared(self, Tmatrix, sharedArray):
        sharedArray[0] = Tmatrix[0,0]; sharedArray[1] = Tmatrix[0,1]; sharedArray[2] = Tmatrix[0,2]; 
        sharedArray[3] = Tmatrix[1,0]; sharedArray[4] = Tmatrix[1,1]; sharedArray[5] = Tmatrix[1,2]; 
        sharedArray[6] = Tmatrix[2,0]; sharedArray[7] = Tmatrix[2,1]; sharedArray[8] = Tmatrix[2,2]; 
    
    def convert_numpyArray_to_shared(self, np_array, sharedArray, size):
        for i in range(0, size):
            sharedArray[i] = np_array[i]
    
    def convert_shared_to_numpyArray(self, sharedArray, size):
        numpyArray = np.empty(size)
        for i in range(0, size):
            numpyArray[i] = sharedArray[i]
        return sharedArray