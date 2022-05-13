
import open3d as o3d
import numpy as np
import os
import math
from pyntcloud import PyntCloud
from pandas import DataFrame
import scipy.linalg as linalg

#from mayavi import mlab # for

import matplotlib.pyplot as plt

def pick_points(pcd):
    print("")
    print(
        "1) Please pick at least three correspondences using [shift + left click]"
    )
    print("   Press [shift + right click] to undo point picking")
    print("2) After picking points, press 'Q' to close the window")
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window()
    vis.add_geometry(pcd)
    vis.run()  # user picks points
    vis.destroy_window()
    print("")
    picked_points = vis.get_picked_points()

    return picked_points


# convert to grid with multiple points in a grid
def voxel_conv(point_cloud, leaf_size):
    grid_points = []
    # boundary
    x_min, y_min, z_min = np.amin(point_cloud, axis=0) 
    x_max, y_max, z_max = np.amax(point_cloud, axis=0)
 
    # dimension of voxel grid
    Dx = (x_max - x_min)//leaf_size + 1
    Dy = (y_max - y_min)//leaf_size + 1
    Dz = (z_max - z_min)//leaf_size + 1
    print("Dx x Dy x Dz is {} x {} x {}".format(Dx, Dy, Dz))
 
    # index of voxel
    h = list()  
    for i in range(len(point_cloud)):
        hx = (point_cloud[i][0] - x_min)//leaf_size
        hy = (point_cloud[i][1] - y_min)//leaf_size
        hz = (point_cloud[i][2] - z_min)//leaf_size
        h.append(hx + hy*Dx + hz*Dx*Dy)
    h = np.array(h)
 
    h_indice = np.argsort(h) # index range small-large 
    h_sorted = h[h_indice]
    begin = 0
    grid_points_temp = []
    grid_height_temp = 0
    grid_height = []

    np.append(h_sorted, -1)
    for i in range(len(h_sorted)-1):   # 0~9999
        indexinpc = h_indice[i]

        if h_sorted[i] == h_sorted[i + 1]:
            grid_points_temp.append(point_cloud[indexinpc])
            grid_height_temp = grid_height_temp + point_cloud[indexinpc][2]
            
        else:
            grid_points_temp.append(point_cloud[indexinpc])
            grid_height_temp = grid_height_temp + point_cloud[indexinpc][2]
            grid_height_temp = grid_height_temp/(i-begin)
            grid_height.append(grid_height_temp)
            grid_points.append(grid_points_temp)
            begin = i
            grid_points_temp = []
            grid_height_temp = 0

        # if h_sorted[i] == h_sorted[i + 1]:
        #     grid_points_temp.append(point_cloud[indexinpc])
        #     if point_cloud[indexinpc][2]>grid_height_temp:
        #         grid_height_temp = point_cloud[indexinpc][2]
        #     else:
        #         grid_height_temp = grid_height_temp
            
        # else:
        #     grid_points_temp.append(point_cloud[indexinpc])
        #     if point_cloud[indexinpc][2]>grid_height_temp:
        #         grid_height_temp = point_cloud[indexinpc][2]
        #     else:
        #         grid_height_temp = grid_height_temp
        #     #grid_height_temp = grid_height_temp/(i-begin)
        #     grid_height.append(grid_height_temp)
        #     grid_points.append(grid_points_temp)
        #    # begin = i
        #     grid_points_temp = []
        #     grid_height_temp = 0

    return grid_points, grid_height #return list

# select points in the specific circle
def InnerCircle(point_cloud,center_point,diameter):
    In_circle = []
    
    for i in range(len(point_cloud)):
        dist = math.sqrt((point_cloud[i][0] - center_point[0])**2 + (point_cloud[i][1] - center_point[1])**2)
        if dist <= diameter/2:
            In_circle.append(point_cloud[i])
            
    return In_circle #list

# euler rotation
def rotate_mat(axis, radian):
    rot_matrix = linalg.expm(np.cross(np.eye(3), axis / linalg.norm(axis) * radian))
    return rot_matrix


def column_conv(point_cloud, leaf_size):
    grid_points = []

    x_min, y_min, z_min = np.amin(point_cloud, axis=0)
    x_max, y_max, z_max = np.amax(point_cloud, axis=0)
 
    Dx = (x_max - x_min)//leaf_size + 1
    Dy = (y_max - y_min)//leaf_size + 1
    print("Dx x Dy x Dz is {} x {} ".format(Dx, Dy))
 
  
    h = list() 
    for i in range(len(point_cloud)):
        hx = (point_cloud[i][0] - x_min)//leaf_size
        hy = (point_cloud[i][1] - y_min)//leaf_size
        
        h.append(hx + hy*Dx)
    h = np.array(h)
 

    h_indice = np.argsort(h)
    h_sorted = h[h_indice]
    begin = 0
    column_points = []
    np.append(h_sorted,-1)
    for i in range(len(h_sorted)-1):  
        if h_sorted[i] == h_sorted[i + 1]:

            continue
        else:
            point_idx = h_indice[begin: i + 1]
            column_points.append(np.mean(point_cloud[point_idx], axis=0))
            begin = i

    column_points = np.array(column_points, dtype=np.float64)
    
    return column_points 


def depthcalc(point_cloud, volumeR, leaf_size):

    column_points = column_conv(point_cloud, leaf_size)
    
    columnarea = leaf_size**2
    zsum = 0
    for i in range(len(column_points)-1): 
        zsum = zsum + column_points[i, 2]

    h = (zsum - volumeR/columnarea) / len(column_points)

    return h

def depthcalcbyh(point_cloud, H, zmax):

    zsum = 0
    for i in range (len(point_cloud)): 
        zsum = zsum + point_cloud[i][2]
    zmean = zsum/len(point_cloud)

    Depth = zmax-zmean + H

    return Depth



def eulerAnglesToRotationMatrix(theta) :
    
    R_x = np.array([[1,         0,                  0                   ],
                    [0,         math.cos(theta[0]), -math.sin(theta[0]) ],
                    [0,         math.sin(theta[0]), math.cos(theta[0])  ]
                    ])
                       
    R_y = np.array([[math.cos(theta[1]),    0,      math.sin(theta[1])  ],
                    [0,                     1,      0                   ],
                    [-math.sin(theta[1]),   0,      math.cos(theta[1])  ]
                    ])
                
    R_z = np.array([[math.cos(theta[2]),    -math.sin(theta[2]),    0],
                    [math.sin(theta[2]),    math.cos(theta[2]),     0],
                    [0,                     0,                      1]
                    ])
                                       
    R = np.dot(R_z, np.dot( R_y, R_x ))

    return R

def ismember(M,element, axis='row'):
    indexinA = np.argwhere(M==element)
    sizeB = np.size(element)
    if axis == 'row':
        indexinAB = [i[0] for i in indexinA]
        indexinAD = dict(Counter(indexinAB))
        RepValue = [key for key,value in indexinAD.items() if value ==sizeB ]

    if axis == 'single':
        print('single')

    return RepValue



if __name__ == '__main__':

    #source_dir = "../11_Oct/cam1/pcd/1633939443632570.pcd"
    source_dir = "./"
    pcd_source = o3d.io.read_point_cloud(os.path.join(source_dir,"cam1_1.pcd"))
    #pcd_source.paint_uniform_color([1,0.706,0]) #yellow
    #o3d.visualization.draw_geometries([pcd_source])
    np_source = np.asarray(pcd_source.points)
    x = np_source[:,0]
    y = np_source[:,1]
    z = np_source[:,2]
    print("minmaxx  minmaxy minmaxz is {} , {} , {}, {} , {} , {}".format(np_source[:,0].min(),np_source[:,0].max(),np_source[:,1].min(),np_source[:,1].max(),np_source[:,2].min(),np_source[:,2].max()))


    pick_points(pcd_source)


    #The region we wanted using x-axis and y-axis
    #Like a bird's eye view
    bounding_ploy = np.array([
                              [-0.19, -0.04, 0],
                              [0.12, -0.04, 0],
                              [0.12, 0.16, 0],
                              [-0.19, 0.16, 0]
                             ], dtype = np.float32).reshape([-1, 3]).astype("float64")


    print(bounding_ploy)

    bounding_polygon = np.array(bounding_ploy, dtype = np.float64)

    vol = o3d.visualization.SelectionPolygonVolume()

    #The Z-axis is used to define the height of the selected region
    vol.orthogonal_axis = "Z"
    vol.axis_max = np_source[:,2].max()
    vol.axis_min =np_source[:,2].min()


    vol.bounding_polygon = o3d.utility.Vector3dVector(bounding_polygon)
    comp = vol.crop_point_cloud(pcd_source)
    print(comp)
    xyz_load = np.asarray(comp.points)
    #pcd_source.paint_uniform_color([1,0.706,0]) #yellow
    #comp.paint_uniform_color([0,0.651,0.929])#blue
    o3d.visualization.draw_geometries([comp])
    print(xyz_load[:,0].min(),xyz_load[:,0].max(),xyz_load[:,1].min(),xyz_load[:,1].max(),xyz_load[:,2].min(),xyz_load[:,2].max())

    plane_model, inliers = comp.segment_plane(distance_threshold=0.022,
                                            ransac_n=9,
                                            num_iterations=800)
    [a, b, c, d] = plane_model
    #print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")
    #a=0.05
    #b=0.47
    #c=0.88 
    #d=-0.65
    theta = math.acos(-c/(np.sqrt(a**2+b**2+c**2)))
    print(theta)

    axis_x, axis_y, axis_z = [1,0,0], [0,1,0], [0,0,1]

    rot_matrix = rotate_mat(axis_y, theta)


    inlier_cloud = comp.select_by_index(inliers)
    #inlier_cloud.paint_uniform_color([1.0, 0, 0])
    outlier_cloud = comp.select_by_index(inliers, invert=True)
    #outlier_cloud.paint_uniform_color([0, 1, 0])
    #o3d.visualization.draw_geometries([inlier_cloud])
    np_inlier = np.asarray(inlier_cloud.points)
    #save the cropped region
    #xyz_inlier = np.asarray(inlier_cloud.points)
    #np.savetxt( '1_' + '.txt', xyz_inlier)
    bounding_ploy1 = np.array([
                        [-0.16, -0.02, 0],
                        [0.09, -0.02, 0],
                        [0.09, 0.13, 0],
                        [-0.16, 0.13, 0]
                        ], dtype = np.float32).reshape([-1, 3]).astype("float64")

    bounding_polygon1 = np.array(bounding_ploy1, dtype = np.float64)

    vol = o3d.visualization.SelectionPolygonVolume()

    #The Z-axis is used to define the height of the selected region
    vol.orthogonal_axis = "Z"
    vol.axis_max = np_inlier[:,2].max()
    vol.axis_min =np_inlier[:,2].min()


    vol.bounding_polygon = o3d.utility.Vector3dVector(bounding_polygon1)
    compS = vol.crop_point_cloud(inlier_cloud)

    np_inlier = np.asarray(compS.points)
    

    print(np_inlier[:,0].min(),np_inlier[:,0].max(),np_inlier[:,1].min(),np_inlier[:,1].max(),np_inlier[:,2].min(),np_inlier[:,2].max())

    np_inlieruni = np.dot(rot_matrix, np_inlier.T)
    np_inlieruni = np_inlieruni.T

    inlier_pointGrid, inlier_pointNum = voxel_conv(np_inlieruni, 0.05)


    Max_grid = inlier_pointGrid[inlier_pointNum.index(max(inlier_pointNum))]
    Max_grid = np.array(Max_grid, dtype=np.float64)
    print(Max_grid[:,2].max())
    print(Max_grid[np.argmax(Max_grid[:,2])])
    center_point = Max_grid[np.argmax(Max_grid[:,2])]


    InnerCircle = InnerCircle(np_inlier,center_point,0.3)

    InnerCircle = np.array(InnerCircle, dtype=np.float64)
    #print(center_point)

    Depth = depthcalc(InnerCircle, 0.00016, 0.01)


    xs = InnerCircle[:, 0]
    ys = InnerCircle[:, 1]
    zs = InnerCircle[:, 2]
    print(xs)
    fig = plt.figure(figsize=(12,7))
    ax = fig.add_subplot(projection='3d')
    img = ax.scatter(xs, ys, zs, cmap=plt.hot())
    fig.colorbar(img)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    plt.show()





