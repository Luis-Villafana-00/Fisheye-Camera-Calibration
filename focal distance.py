import cv2, numpy as np, matplotlib.pyplot as plt, os, numpy.linalg as lalg
from scipy.interpolate import CubicSpline as cubic_spline
import utilities as ut, json
from open3d.visualization import draw_geometries as dg
from open3d.utility import Vector3dVector as v3d
from open3d.utility import Vector2iVector as v2i
from open3d.geometry import LineSet as o3dls
from matplotlib.collections import LineCollection as lc

plt.rc('legend', fontsize = 14)

delta_1 = 3333
width = 3.45
n_map = 111111
image_alpha = .234
mesh_alpha = .789
arc_alpha = .555
size = 55
blue = np.array([0, 137, 250]) / 255
red = np.array([255, 43, 135]) / 255
black = red * 0
gray = black + .987
pattern_size = (7, 6)

'''load data and detect corners'''
parameters_path = 'camera parameters/2025/'
image_path = 'images/4k fisheye/for dewarping/'
names = os.listdir(image_path) 
image_names = ['0.jpg', '1.jpg', '2.jpg', '3.jpg', '4.jpg', '5.jpg']
mask_names = {'1.jpg' : '1 mask.png', '2.jpg' : '2 mask.png',
              '3.jpg' : '3 mask.png', '4.jpg' : '4 mask.png'}

with open(parameters_path + 'parameters.json', 'r') as file:
    parameters = json.load(file)

O1 = np.array(parameters['O1'])
O2 = np.array(parameters['O2'])
K = np.array(parameters['K'])
map_x = np.load(parameters_path + 'map x.npy')
map_y = np.load(parameters_path + 'map y.npy')
N = len(K)
n = np.arange(0, N)
B = np.diag(ut.binomial_coefficients(N))
B[[0, 1], [0, 1]] = [0, 0]
warped_radii = np.linspace(0, parameters['rho 1'], n_map)
normalized_radii = warped_radii / parameters['rho 1']
R = (normalized_radii[:, None] ** n) * ((1 - normalized_radii[:, None]) ** (N -1 - n))
distortions = R @ B @ K[:, None]
dewarped_radii = warped_radii + distortions[:, 0]
dewarp = cubic_spline(warped_radii, dewarped_radii)
image_shape = np.flip(map_x.shape)
flag = cv2.CALIB_ZERO_TANGENT_DIST | cv2.CALIB_FIX_K1 | cv2.CALIB_FIX_K2 | cv2.CALIB_FIX_K3

grids = []
images = []
for i, image_name in enumerate(image_names):
    loaded_image = 255 - cv2.imread(image_path + image_name, 0)
    image_y, image_x = np.mgrid[0 : loaded_image.shape[0], 0 : loaded_image.shape[1]]
    images.append(loaded_image)
    
    '''36, 28, 237'''
    mask = None
    if image_name in mask_names:
        mask_name = mask_names[image_name]
        mask = cv2.imread(image_path + mask_name)
        mask = (mask[:, :, 0] == 36) * (mask[:, :, 1] == 28) * (mask[:, :, 2] == 237)
    
    grid = ut.detect_corners(loaded_image, pattern_size = pattern_size, mask = mask)
    grid = np.array(grid).reshape(pattern_size[:: -1] + tuple([2])).transpose((1, 0 ,2))
    grids.append(grid)

'''dewarp corners'''
dewarped_grids = []
axes = []
for l, (grid, image) in enumerate(zip(grids, images)):
    dewarped_grid = grid.reshape((-1, 2)) - O1
    r = lalg.norm(dewarped_grid, axis = 1)
    v = dewarped_grid / r[:, None]
    r = dewarp(r)
    dewarped_grid = v * r[:, None] + O2
    dewarped_grid = dewarped_grid.reshape(grid.shape)
    dewarped_grids.append(dewarped_grid)
    
    '''grid refinement'''
    dewarped_grid = ut.refine_grid(dewarped_grid)
    
    remapped_image = cv2.remap(image.astype('float32'), map_x, map_y, cv2.INTER_LINEAR,
                               borderValue = np.inf)
    #plt.figure('remapped image ' + str(l))
    _, axis = plt.subplots()
    axes.append(axis)
    axis.set_xticks([])
    axis.set_yticks([])
    axis.imshow(remapped_image, cmap = 'gray', alpha = image_alpha)

    segments = []
    for j in [0, 1]:
        for arc in dewarped_grid:
            segments += [np.array([arc[0], arc[-1]])]
        dewarped_grid = dewarped_grid.transpose((1, 0, 2))
    axis.add_collection(lc(segments, color = red, linewidths = width / 1.345))


world_grid = np.mgrid[0 : dewarped_grid.shape[0], 0 : dewarped_grid.shape[1]].transpose((1, 2, 0)) * delta_1
world_grid = world_grid[:, :, [1, 0, 0]] * [[[1, 1, 0]]]

image_corners = []
world_corners = []
for j in range(len(dewarped_grids)):
    image_corners.append(dewarped_grids[j].reshape((-1, 2)).astype('float32'))
    world_corners.append(world_grid.reshape((-1, 3)).astype('float32'))

_, _, _, rvecs, tvecs = cv2.calibrateCamera(world_corners, image_corners,
                                                   image_shape, None, None)

_, mtx, _, rvecs, tvecs = cv2.calibrateCamera(world_corners, image_corners,
                                                   image_shape, None, None, flags = flag,
                                                   rvecs = rvecs, tvecs = tvecs)

rotation_matrices = [cv2.Rodrigues(j)[0] for j in rvecs]
translations = [j[:, 0] for j in tvecs]

n = np.array([0, 0, 1])
f = np.mean([mtx[0, 0], mtx[1, 1]])
O = np.array([mtx[0, 2], mtx[1, 2], 0])
plane_point =  np.array([0, 0, f]) + O
world_grid = world_grid.reshape((-1, 3))
E = []
for i, (image_grid, rotation, translation) in enumerate(zip(dewarped_grids, rotation_matrices, translations)):
    grid = image_grid[:, :, [0, 1, 1]] * [[[1, 1, 0]]] - O + [[[0, 0, f]]]
    image_meshes = ut.create_meshes(grid, black, gray)
    image_grid_lines = ut.create_lines(grid)
    grid = grid.reshape((-1, 3))
    
    transformed_points = world_grid @ rotation.T + translation
    transformed_grid = transformed_points.reshape(pattern_size + tuple([3]))
    world_meshes = ut.create_meshes(transformed_grid, black, gray)
    world_grid_lines = ut.create_lines(transformed_grid)

    line_points = np.concatenate((transformed_points, [[0, 0, 0]]), axis = 0)
    line_edges = np.arange(0, len(line_points) - 1)[:, None][:, [0, 0]]
    line_edges[:, 1] = len(line_edges)
    transformed_lines = o3dls(v3d(line_points), v2i(line_edges))
    transformed_lines.paint_uniform_color(gray * .678)
    
    transformed_u = ([0, 0, 0] - transformed_points) / lalg.norm([0, 0, 0] - transformed_points, axis = 1)[:, None]
    t = np.sum(n * (plane_point - transformed_points), axis = 1) / np.sum(n * transformed_u, axis = 1)
    reprojected_grid = transformed_points + transformed_u * t[:, None]
    reprojection_errors = lalg.norm(reprojected_grid - grid, axis = 1)
    E.append(reprojection_errors)
    
    axes[i].scatter(reprojected_grid[:, 0] + O[0], reprojected_grid[:, 1] + O[1], color = black, zorder = 3)
    axes[i].legend(['Image Checkerboard', 'Reprojected World Points'])
    
    dg([transformed_lines] + image_grid_lines + world_grid_lines + image_meshes + world_meshes,
       mesh_show_back_face = True)

E = np.array(E).flatten()
#plt.plot(np.sort(E))
print('Mean Reprojection Error:')
print(np.mean(E))
print('Mean Reprojection Percent:')
print(np.mean(E) / remapped_image.shape[1] * 100)

parameters['camera matrix'] = mtx.tolist()

with open(parameters_path + 'parameters.json', 'w') as file:
    json.dump(parameters, file, indent = 4)