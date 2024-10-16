import cv2
import numpy as np

def rodrigues_vec_to_rotation_mat(rodrigues_vec): 
     return cv2.Rodrigues(rodrigues_vec)[0] 

print(rodrigues_vec_to_rotation_mat(np.array([[-0.59690669],
       [-0.08682592],
       [ 0.00397184]])))