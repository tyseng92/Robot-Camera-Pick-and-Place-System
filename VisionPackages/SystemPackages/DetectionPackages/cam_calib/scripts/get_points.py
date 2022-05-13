import open3d as o3d
import numpy as np
import math
from scipy.spatial.transform import Rotation as R

# link 1: https://learnopencv.com/rotation-matrix-to-euler-angles/
# link 2: https://mathematica.stackexchange.com/questions/106257/how-do-i-get-the-inverse-of-a-homogeneous-transformation-matrix
# link 3: https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.transform.Rotation.as_euler.html
# link 4: http://www.open3d.org/docs/latest/tutorial/Basic/transformation.html


# Checks if a matrix is a valid rotation matrix.
def isRotationMatrix(R) :
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype = R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6

# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).
def rotationMatrixToEulerAngles(R) :

    assert(isRotationMatrix(R))

    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])

    singular = sy < 1e-6

    if  not singular :
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else :
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0

    return np.array([x, y, z])

#pcd = o3d.io.read_point_cloud("astra_calib_cam01.pcd")
pcd = o3d.io.read_point_cloud("calib.pcd")
#mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
#        size=1.0, origin=[0, 0, 0])

vis = o3d.visualization.VisualizerWithEditing()
vis.create_window()
vis.add_geometry(pcd)
#vis.add_geometry(mesh_frame)
vis.run()
vis.destroy_window()
print(vis.get_picked_points())
index = vis.get_picked_points()
pts = []
for i in index:
    pts.append(pcd.points[i])
    print(pcd.points[i])
    #print(type(pcd.points[i]))
print(pts)

pt1 = pts[0]
pt2 = pts[1]
pt3 = pts[2]

# point 1 as origin, x direction from point 1 to point 2, point 3 is any point on the plane
vec12 = pt2 - pt1
vec13 = pt3 - pt1

# x axis
uvec12 = vec12 / np.linalg.norm(vec12)
uvec13 = vec13 / np.linalg.norm(vec13)

normvec = np.cross(uvec12, uvec13)

# z axis, 'u' means unit vector
unormvec = normvec / np.linalg.norm(normvec)

# 'p' means plane (order of argument in cross product is important, from z axis to x axis)
pnormvec = np.cross(unormvec, uvec12) 

# y axis
upnormvec = pnormvec / np.linalg.norm(pnormvec)

xaxis = uvec12
yaxis = upnormvec
zaxis = unormvec

print(xaxis)
print(type(xaxis))

#rot_mat = np.matrix([xaxis, yaxis, zaxis]).transpose()
# get inverse of the matrix, dont use transpose since it is transposed already
rot_mat = np.matrix([xaxis, yaxis, zaxis])

print(rot_mat)
print(type(rot_mat))

euler_angles = rotationMatrixToEulerAngles(rot_mat)

inv_pt1 = -1 * rot_mat.dot(pt1)

print(inv_pt1.shape)
print("translation: " + str(inv_pt1))
print("rotation: " + str(euler_angles))

# test scipy
r = R.from_matrix([xaxis, yaxis, zaxis])
# extrinsic angle (fixed angle: sXYZ) 
euler_angle_sp = r.as_euler('xyz')
print("scipy euler angles" + str(euler_angle_sp))

# draw geometry to confirm
origin_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
        size=0.2, origin=[0,0,0])
mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
        size=0.2, origin=pt1)
mesh_frame.rotate(np.asarray(rot_mat.transpose()), center=True) 

pcd.estimate_normals()
distances = pcd.compute_nearest_neighbor_distance()
avg_dist = np.mean(distances)
radius = 1.5 * avg_dist
#pcd.orient_normals_towards_camera_location(pcd.get_center())
mesh_pcd = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
           pcd,
           o3d.utility.DoubleVector([radius, radius * 2]))

o3d.visualization.draw_geometries([mesh_pcd + mesh_frame + origin_frame])

