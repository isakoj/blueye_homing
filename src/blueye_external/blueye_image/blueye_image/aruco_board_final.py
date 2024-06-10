import numpy as np


# Length of the marker sides
marker_length_huge = 0.222
marker_length_big = 0.15
marker_length_mid = 0.1068
marker_length_small = 0.071
marker_length_mini = 0.07

# Rotation matrix +180 deg around the y-axis
rot_matrix_180_y = np.array([[-1, 0, 0],
                             [0, 1, 0],
                             [0, 0, -1]
                            ])

rot_matrix_180_x = np.array([[1, 0, 0],
                             [0, -1, 0],
                             [0, 0, -1]
                            ])

rot_matrix_180_z = np.array([[-1, 0, 0],
                             [0, -1, 0],
                             [0, 0, 1]
                            ])

rot_matrix_90_z = np.array([[0, -1, 0],
                            [1, 0, 0],
                            [0, 0, 1]])

rot_matrix_neg_90_z = np.array([[0, 1, 0],
                                [-1, 0, 0],
                                [0, 0, 1]])



# This is the rotation that transforms F_board into F_dock
R_board_dock = rot_matrix_180_y

# Tag pos (midpoint) wrt dock frame expressed in dock frame. AKA t_dock_tag_list
# Find these from Blender (x, y, z)
# IDs from 0 to 20     
t_dock_tag_list = [ np.array([1.972, 1.46, 1.176], dtype = 'float32'),
                    np.array([1.4925, 1.9199, 1.176], dtype = 'float32'),
                    np.array([1.012, 1.46, 1.176], dtype = 'float32'),
                    np.array([1.4925, 0.51675, 1.176], dtype = 'float32'),
                    np.array([1.4925, 1.61, 2.456], dtype = 'float32'),
                    np.array([1.972, 1.4364, 2.0964], dtype = 'float32'),
                    np.array([1.012, 1.4364, 2.0964], dtype = 'float32'),
                    np.array([1.972, 1.0447, 1.176], dtype = 'float32'),
                    np.array([1.972, 1.684, 1.176], dtype = 'float32'),
                    np.array([1.782, 1.875, 1.176], dtype = 'float32'),
                    np.array([1.203, 1.875, 1.176], dtype = 'float32'),
                    np.array([1.012, 1.684, 1.176], dtype = 'float32'),
                    np.array([1.012, 1.0447, 1.176], dtype = 'float32'),
                    np.array([0.052576, 1.0447, 1.176], dtype = 'float32'),
                    np.array([1.5325, 1.7317, 2.456], dtype = 'float32'),
                    np.array([1.4525, 1.7317, 2.456], dtype = 'float32'),
                    np.array([2.0181, 1.8699, 1.7], dtype = 'float32'),
                    np.array([0.555, 1.8756, 1.176], dtype = 'float32'),
                    np.array([0.008167, 1.8699, 1.7], dtype = 'float32'),
                    np.array([0.22912, 1.8745, 2.1781], dtype = 'float32'),
                    np.array([1.0038, 1.8745, 2.1781], dtype = 'float32'),
                    ]

# The origo of the board is located in the middle of tag 1
t_board_tag0 = R_board_dock @ (t_dock_tag_list[0] - t_dock_tag_list[1])
t_board_tag1 = R_board_dock @ (t_dock_tag_list[1] - t_dock_tag_list[1])
t_board_tag2 = R_board_dock @ (t_dock_tag_list[2] - t_dock_tag_list[1])
t_board_tag3 = R_board_dock @ (t_dock_tag_list[3] - t_dock_tag_list[1])
t_board_tag4 = R_board_dock @ (t_dock_tag_list[4] - t_dock_tag_list[1])
t_board_tag5 = R_board_dock @ (t_dock_tag_list[5] - t_dock_tag_list[1])
t_board_tag6 = R_board_dock @ (t_dock_tag_list[6] - t_dock_tag_list[1])
t_board_tag7 = R_board_dock @ (t_dock_tag_list[7] - t_dock_tag_list[1])
t_board_tag8 = R_board_dock @ (t_dock_tag_list[8] - t_dock_tag_list[1])
t_board_tag9 = R_board_dock @ (t_dock_tag_list[9] - t_dock_tag_list[1])
t_board_tag10 = R_board_dock @ (t_dock_tag_list[10] - t_dock_tag_list[1])
t_board_tag11 = R_board_dock @ (t_dock_tag_list[11] - t_dock_tag_list[1])
t_board_tag12 = R_board_dock @ (t_dock_tag_list[12] - t_dock_tag_list[1])
t_board_tag13 = R_board_dock @ (t_dock_tag_list[13] - t_dock_tag_list[1])
t_board_tag14 = R_board_dock @ (t_dock_tag_list[14] - t_dock_tag_list[1])
t_board_tag15 = R_board_dock @ (t_dock_tag_list[15] - t_dock_tag_list[1])
t_board_tag16 = R_board_dock @ (t_dock_tag_list[16] - t_dock_tag_list[1])
t_board_tag17 = R_board_dock @ (t_dock_tag_list[17] - t_dock_tag_list[1])
t_board_tag18 = R_board_dock @ (t_dock_tag_list[18] - t_dock_tag_list[1])
t_board_tag19 = R_board_dock @ (t_dock_tag_list[19] - t_dock_tag_list[1])
t_board_tag20 = R_board_dock @ (t_dock_tag_list[20] - t_dock_tag_list[1])


# Define and set up the aruco board. Distances is relative to the center of Tag 1, which is origo of the board 
# Copy line 4 and change only t_board_tag(i) 
pos_board = [np.array([(t_board_tag0[0] - marker_length_big/2, t_board_tag0[1] + marker_length_big/2, t_board_tag0[2]), (t_board_tag0[0] + marker_length_big/2, t_board_tag0[1] + marker_length_big/2, t_board_tag0[2]), (t_board_tag0[0] + marker_length_big/2, t_board_tag0[1] - marker_length_big/2, t_board_tag0[2]), (t_board_tag0[0] - marker_length_big/2, t_board_tag0[1] - marker_length_big/2, t_board_tag0[2])], dtype = 'float32'), 
             np.array([(-marker_length_big/2, marker_length_big/2, 0), (marker_length_big/2, marker_length_big/2, 0), (marker_length_big/2, -marker_length_big/2, 0), (-marker_length_big/2, -marker_length_big/2, 0)], dtype = 'float32'),
             np.array([(t_board_tag2[0] - marker_length_big/2, t_board_tag2[1] + marker_length_big/2, t_board_tag2[2]), (t_board_tag2[0] + marker_length_big/2, t_board_tag2[1] + marker_length_big/2, t_board_tag2[2]), (t_board_tag2[0] + marker_length_big/2, t_board_tag2[1] - marker_length_big/2, t_board_tag2[2]), (t_board_tag2[0] - marker_length_big/2, t_board_tag2[1] - marker_length_big/2, t_board_tag2[2])], dtype = 'float32'),
             np.array([(t_board_tag3[0] - marker_length_huge/2, t_board_tag3[1] + marker_length_huge/2, t_board_tag3[2]), (t_board_tag3[0] + marker_length_huge/2, t_board_tag3[1] + marker_length_huge/2, t_board_tag3[2]), (t_board_tag3[0] + marker_length_huge/2, t_board_tag3[1] - marker_length_huge/2, t_board_tag3[2]), (t_board_tag3[0] - marker_length_huge/2, t_board_tag3[1] - marker_length_huge/2, t_board_tag3[2])], dtype = 'float32'), 
             np.array([(t_board_tag4[0] - marker_length_big/2, t_board_tag4[1] + marker_length_big/2, t_board_tag4[2]), (t_board_tag4[0] + marker_length_big/2, t_board_tag4[1] + marker_length_big/2, t_board_tag4[2]), (t_board_tag4[0] + marker_length_big/2, t_board_tag4[1] - marker_length_big/2, t_board_tag4[2]), (t_board_tag4[0] - marker_length_big/2, t_board_tag4[1] - marker_length_big/2, t_board_tag4[2])], dtype = 'float32'), 
             np.array([(t_board_tag5[0] - marker_length_mid/2, t_board_tag5[1] + marker_length_mid/2, t_board_tag5[2]), (t_board_tag5[0] + marker_length_mid/2, t_board_tag5[1] + marker_length_mid/2, t_board_tag5[2]), (t_board_tag5[0] + marker_length_mid/2, t_board_tag5[1] - marker_length_mid/2, t_board_tag5[2]), (t_board_tag5[0] - marker_length_mid/2, t_board_tag5[1] - marker_length_mid/2, t_board_tag5[2])], dtype = 'float32'),
             np.array([(t_board_tag6[0] - marker_length_mid/2, t_board_tag6[1] + marker_length_mid/2, t_board_tag6[2]), (t_board_tag6[0] + marker_length_mid/2, t_board_tag6[1] + marker_length_mid/2, t_board_tag6[2]), (t_board_tag6[0] + marker_length_mid/2, t_board_tag6[1] - marker_length_mid/2, t_board_tag6[2]), (t_board_tag6[0] - marker_length_mid/2, t_board_tag6[1] - marker_length_mid/2, t_board_tag6[2])], dtype = 'float32'),  
             np.array([(t_board_tag7[0] - marker_length_small/2, t_board_tag7[1] + marker_length_small/2, t_board_tag7[2]), (t_board_tag7[0] + marker_length_small/2, t_board_tag7[1] + marker_length_small/2, t_board_tag7[2]), (t_board_tag7[0] + marker_length_small/2, t_board_tag7[1] - marker_length_small/2, t_board_tag7[2]), (t_board_tag7[0] - marker_length_small/2, t_board_tag7[1] - marker_length_small/2, t_board_tag7[2])], dtype = 'float32'), 
             np.array([(t_board_tag8[0] - marker_length_small/2, t_board_tag8[1] + marker_length_small/2, t_board_tag8[2]), (t_board_tag8[0] + marker_length_small/2, t_board_tag8[1] + marker_length_small/2, t_board_tag8[2]), (t_board_tag8[0] + marker_length_small/2, t_board_tag8[1] - marker_length_small/2, t_board_tag8[2]), (t_board_tag8[0] - marker_length_small/2, t_board_tag8[1] - marker_length_small/2, t_board_tag8[2])], dtype = 'float32'), 
             np.array([(t_board_tag9[0] - marker_length_small/2, t_board_tag9[1] + marker_length_small/2, t_board_tag9[2]), (t_board_tag9[0] + marker_length_small/2, t_board_tag9[1] + marker_length_small/2, t_board_tag9[2]), (t_board_tag9[0] + marker_length_small/2, t_board_tag9[1] - marker_length_small/2, t_board_tag9[2]), (t_board_tag9[0] - marker_length_small/2, t_board_tag9[1] - marker_length_small/2, t_board_tag9[2])], dtype = 'float32'), 
             np.array([(t_board_tag10[0] - marker_length_small/2, t_board_tag10[1] + marker_length_small/2, t_board_tag10[2]), (t_board_tag10[0] + marker_length_small/2, t_board_tag10[1] + marker_length_small/2, t_board_tag10[2]), (t_board_tag10[0] + marker_length_small/2, t_board_tag10[1] - marker_length_small/2, t_board_tag10[2]), (t_board_tag10[0] - marker_length_small/2, t_board_tag10[1] - marker_length_small/2, t_board_tag10[2])], dtype = 'float32'),
             np.array([(t_board_tag11[0] - marker_length_small/2, t_board_tag11[1] + marker_length_small/2, t_board_tag11[2]), (t_board_tag11[0] + marker_length_small/2, t_board_tag11[1] + marker_length_small/2, t_board_tag11[2]), (t_board_tag11[0] + marker_length_small/2, t_board_tag11[1] - marker_length_small/2, t_board_tag11[2]), (t_board_tag11[0] - marker_length_small/2, t_board_tag11[1] - marker_length_small/2, t_board_tag11[2])], dtype = 'float32'),  
             np.array([(t_board_tag12[0] - marker_length_small/2, t_board_tag12[1] + marker_length_small/2, t_board_tag12[2]), (t_board_tag12[0] + marker_length_small/2, t_board_tag12[1] + marker_length_small/2, t_board_tag12[2]), (t_board_tag12[0] + marker_length_small/2, t_board_tag12[1] - marker_length_small/2, t_board_tag12[2]), (t_board_tag12[0] - marker_length_small/2, t_board_tag12[1] - marker_length_small/2, t_board_tag12[2])], dtype = 'float32'),  
             np.array([(t_board_tag13[0] - marker_length_small/2, t_board_tag13[1] + marker_length_small/2, t_board_tag13[2]), (t_board_tag13[0] + marker_length_small/2, t_board_tag13[1] + marker_length_small/2, t_board_tag13[2]), (t_board_tag13[0] + marker_length_small/2, t_board_tag13[1] - marker_length_small/2, t_board_tag13[2]), (t_board_tag13[0] - marker_length_small/2, t_board_tag13[1] - marker_length_small/2, t_board_tag13[2])], dtype = 'float32'),
             np.array([(t_board_tag14[0] - marker_length_mini/2, t_board_tag14[1] + marker_length_mini/2, t_board_tag14[2]), (t_board_tag14[0] + marker_length_mini/2, t_board_tag14[1] + marker_length_mini/2, t_board_tag14[2]), (t_board_tag14[0] + marker_length_mini/2, t_board_tag14[1] - marker_length_mini/2, t_board_tag14[2]), (t_board_tag14[0] - marker_length_mini/2, t_board_tag14[1] - marker_length_mini/2, t_board_tag14[2])], dtype = 'float32'),
             np.array([(t_board_tag15[0] - marker_length_mini/2, t_board_tag15[1] + marker_length_mini/2, t_board_tag15[2]), (t_board_tag15[0] + marker_length_mini/2, t_board_tag15[1] + marker_length_mini/2, t_board_tag15[2]), (t_board_tag15[0] + marker_length_mini/2, t_board_tag15[1] - marker_length_mini/2, t_board_tag15[2]), (t_board_tag15[0] - marker_length_mini/2, t_board_tag15[1] - marker_length_mini/2, t_board_tag15[2])], dtype = 'float32'),
             np.array([(t_board_tag16[0], t_board_tag16[1] + marker_length_huge/2, t_board_tag16[2] - marker_length_huge/2), (t_board_tag16[0], t_board_tag16[1] + marker_length_huge/2, t_board_tag16[2] + marker_length_huge/2), (t_board_tag16[0], t_board_tag16[1] - marker_length_huge/2, t_board_tag16[2] + marker_length_huge/2), (t_board_tag16[0], t_board_tag16[1] - marker_length_huge/2, t_board_tag16[2] - marker_length_huge/2)], dtype = 'float32'),
             np.array([(t_board_tag17[0] - marker_length_huge/2, t_board_tag17[1] + marker_length_huge/2, t_board_tag17[2]), (t_board_tag17[0] + marker_length_huge/2, t_board_tag17[1] + marker_length_huge/2, t_board_tag17[2]), (t_board_tag17[0] + marker_length_huge/2, t_board_tag17[1] - marker_length_huge/2, t_board_tag17[2]), (t_board_tag17[0] - marker_length_huge/2, t_board_tag17[1] - marker_length_huge/2, t_board_tag17[2])], dtype = 'float32'),
             np.array([(t_board_tag18[0], t_board_tag18[1] + marker_length_huge/2, t_board_tag18[2] + marker_length_huge/2), (t_board_tag18[0], t_board_tag18[1] + marker_length_huge/2, t_board_tag18[2] - marker_length_huge/2), (t_board_tag18[0], t_board_tag18[1] - marker_length_huge/2, t_board_tag18[2] - marker_length_huge/2), (t_board_tag18[0], t_board_tag18[1] - marker_length_huge/2, t_board_tag18[2] + marker_length_huge/2)], dtype = 'float32'),
             np.array([(t_board_tag19[0] + marker_length_huge/2, t_board_tag19[1] + marker_length_huge/2, t_board_tag19[2]), (t_board_tag19[0] - marker_length_huge/2, t_board_tag19[1] + marker_length_huge/2, t_board_tag19[2]), (t_board_tag19[0] - marker_length_huge/2, t_board_tag19[1] - marker_length_huge/2, t_board_tag19[2]), (t_board_tag19[0] + marker_length_huge/2, t_board_tag19[1] - marker_length_huge/2, t_board_tag19[2])], dtype = 'float32'), 
             np.array([(t_board_tag20[0] + marker_length_huge/2, t_board_tag20[1] + marker_length_huge/2, t_board_tag20[2]), (t_board_tag20[0] - marker_length_huge/2, t_board_tag20[1] + marker_length_huge/2, t_board_tag20[2]), (t_board_tag20[0] - marker_length_huge/2, t_board_tag20[1] - marker_length_huge/2, t_board_tag20[2]), (t_board_tag20[0] + marker_length_huge/2, t_board_tag20[1] - marker_length_huge/2, t_board_tag20[2])], dtype = 'float32'),
            ]       

# Defines the Ids of the markers in the board
id_board = np.array([[0], [1], [2], [3], [4], [5], [6], [7], [8], [9], [10], [11], [12], [13], [14], [15], [16], [17], [18], [19], [20]])
