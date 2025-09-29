import cv2, numpy as np, matplotlib.pyplot as plt, numpy.linalg as lalg
from scipy.interpolate import CubicSpline as cubic_spline
import utilities as ut, json, open3d as o3d
from matplotlib.collections import LineCollection as lc
from open3d.visualization import draw_geometries as dg
from open3d.utility import Vector3dVector as v3d
from open3d.utility import Vector3iVector as v3i
from open3d.utility import Vector2iVector as v2i
from open3d.geometry import PointCloud as o3dp
from open3d.geometry import LineSet as o3dls

def draw_lines(grid, axes, color):
    segments = []
    for j in [0, 1]:
        for arc in grid:
            segments += [np.array([arc[0], arc[-1]])]
        grid = grid.transpose((1, 0, 2))
    axes.add_collection(lc(segments, color = color, linewidths = width))
    return

A = np.array([[0, 1], [-1, 0]])
delta_1 = 1234
width = 3.45 / 1.345
n_map = 111111
n_interp = 89
n_grid = 13
image_alpha = .234
mesh_alpha = .789
arc_alpha = .555
blue = np.array([0, 137, 250]) / 255
red = np.array([255, 43, 135]) / 255
black = red * 0
gray = black + .987
pattern_shape = (7, 6)
world_grid = np.mgrid[0 : pattern_shape[0], 0 : pattern_shape[1]].transpose((1, 2, 0)) * delta_1
world_grid = (world_grid[:, :, [1, 0, 0]] * [[[1, 1, 0]]]).astype('float32')

'''load data and detect corners'''
parameters_path = 'camera parameters/2025/'
image_path = 'images/4k fisheye/stereo images/'

with open(parameters_path + 'parameters.json', 'r') as file:
    parameters = json.load(file)

camera_matrix = np.array(parameters['camera matrix'])
f = np.mean(camera_matrix[[0, 1], [0, 1]])
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
image_shape = map_x.shape
h, w = image_shape
y, x = np.mgrid[0 : h, 0 : w].astype('float32')

'''load stereo pair'''
left = 255 - cv2.imread(image_path + 'left.jpg', 0)
right = 255 - cv2.imread(image_path + 'right.jpg', 0)

remapped_left = cv2.remap(left, map_x, map_y, cv2.INTER_LINEAR).astype('float32')
remapped_right = cv2.remap(right, map_x, map_y, cv2.INTER_LINEAR).astype('float32')
remapped_left[np.isnan(map_x)] = np.nan
remapped_right[np.isnan(map_x)] = np.nan
image_corners = np.array([[0, 0], [1, 0], [1, 1], [0, 1]]) * [w, h]
image_corners = image_corners.reshape((2, 2, 2)).astype('float32')

'''detect corners'''
left_grid = ut.detect_corners(left, pattern_size = pattern_shape[:: -1])
left_grid = left_grid.reshape(pattern_shape + tuple([2]))
right_grid = ut.detect_corners(right, pattern_size = pattern_shape[:: -1])
right_grid = right_grid.reshape(pattern_shape + tuple([2]))

'''plot fisheye corners'''
figure_1, axes_1 = plt.subplots(nrows = 2, ncols = 2)
figure_2, axes_2 = plt.subplots(nrows = 1, ncols = 2)
figure_3, axes_3 = plt.subplots(nrows = 1, ncols = 1)
for i in axes_1, axes_2, np.array(axes_3):
    for j in i.flat:
        j.set_xticks([])
        j.set_yticks([])

for i, (image, grid, title) in enumerate(zip([left, right], [left_grid, right_grid], ['left', 'right'])):
    axes_1[0, i].imshow(image, cmap = 'gray')#, alpha = image_alpha)
    interpolated_grid = ut.interpolate_grid(grid, n_interp, pattern_shape)
    for j in [0, 1]:
        for k in range(0, len(interpolated_grid), n_interp):
            axes_1[0, i].plot(interpolated_grid[k, :, 0], interpolated_grid[k, :, 1],
                            linewidth = width, color = [blue, red][i])#, alpha = mesh_alpha)
        interpolated_grid = interpolated_grid.transpose((1, 0, 2))

'''
dewarp corners
'''
def dewarp_grid(grid):
    dewarped_grid = grid.reshape((-1, 2)) - O1
    r = lalg.norm(dewarped_grid, axis = 1)
    v = dewarped_grid / r[:, None]
    r = dewarp(r)
    dewarped_grid = v * r[:, None] + O2
    dewarped_grid = dewarped_grid.reshape(grid.shape)
    
    '''grid refinement'''
    for j in range(dewarped_grid.shape[0]):
        for k in range(dewarped_grid.shape[1]):
            p1 = dewarped_grid[j, 0][:, None]
            p2 = dewarped_grid[j, -1][:, None]
            p3 = dewarped_grid[0, k][:, None]
            p4 = dewarped_grid[-1, k][:, None]
            t = (((p2 - p1).T @ A @ (p3 - p1)) / ((p4 - p3).T @ A @ (p2 - p1)))[0] 
            p = p3 + (p4 - p3) * t
            dewarped_grid[j, k] = p[:, 0]
    
    return dewarped_grid

left_grid, right_grid = [dewarp_grid(j) for j in [left_grid, right_grid]]

'''
plot dewarped corners
'''
for i, (grid, image, title) in enumerate(zip([left_grid, right_grid], [remapped_left, remapped_right], ['left dewarped', 'right dewarped'])):
    axes_1[1, i].imshow(image, cmap = 'gray')#, alpha = image_alpha)
    draw_lines(grid, axes_1[1, i], [blue, red][i])

'''find rotations and translations of checkerboards'''

left_rotation, left_translation = ut.pose(left_grid, world_grid, camera_matrix)
right_rotation, right_translation = ut.pose(right_grid, world_grid, camera_matrix)

world_meshes = ut.create_meshes(world_grid, black, gray)
world_lines = ut.create_lines(world_grid)
world_grid = world_grid.reshape((-1, 3))

'''transform an image grid to the camera pose frame'''
def transform_grid(grid, rotation, translation):
    grid = grid[:, :, [0, 1, 1]] * [[[1, 1, 0]]] + [[[0, 0, f]]]
    grid = grid.reshape((-1, 3)) - np.concatenate((camera_matrix[[0, 1], [2, 2]], [0]))
    grid = (grid - translation) @ rotation
    return grid

left_O = -left_translation @ left_rotation
right_O = -right_translation @ right_rotation
transformed_left_grid = transform_grid(left_grid, left_rotation, left_translation)
transformed_right_grid = transform_grid(right_grid, right_rotation,  right_translation)
left_corners = transform_grid(image_corners, left_rotation, left_translation)
right_corners = transform_grid(image_corners, right_rotation, right_translation)
v1 = (left_O - right_O) / lalg.norm(left_O - right_O)
v3 = np.mean([left_rotation[:, 2], right_rotation[:, 2]], axis = 0)
v2 = np.cross(v3, v1)
v3 = np.cross(v1, v2)
rotation = np.array([v1, v2, v3]).T
rotation /= lalg.norm(rotation, axis = 0)[None, :]
O = left_O + rotation[:, 2] * f

left_image_grid = np.array([x, y]).transpose((1, 2, 0))[0 :: n_grid, 0 :: n_grid]
right_image_grid = left_image_grid.copy()
image_shape = left_image_grid.shape[0 : 2]
left_image_grid = transform_grid(left_image_grid, left_rotation, left_translation)
left_image_grid = left_image_grid.reshape(image_shape + tuple([3]))
right_image_grid = transform_grid(right_image_grid, right_rotation, right_translation)
right_image_grid = right_image_grid.reshape(image_shape + tuple([3]))
left_image_colors = remapped_left.copy()[0 :: n_grid, 0 :: n_grid] / 255
right_image_colors = remapped_right.copy()[0 :: n_grid, 0 :: n_grid] / 255
left_image_colors[np.isnan(left_image_colors)] = 1
right_image_colors[np.isnan(right_image_colors)] = 1
def image_mesh(image_grid, image_colors):
    meshes = []
    for j in range(len(image_grid) - 1):
        for k in range(len(image_grid[0]) - 1):
            p1 = image_grid[j, k]
            p2 = image_grid[j, k + 1]
            p3 = image_grid[j + 1, k + 1]
            p4 = image_grid[j + 1, k]
            T = np.array([[0, 1, 2], [0, 2, 3]])
            P = np.array([p1, p2, p3, p4])
            triangle_mesh = o3d.geometry.TriangleMesh()
            triangle_mesh.vertices = v3d(P)
            triangle_mesh.triangles = v3i(T)
            colors = image_colors[[j, j, j + 1, j + 1],
                                       [k, k + 1, k + 1, k]][:, None].repeat(3, axis = 1)
            triangle_mesh.vertex_colors = v3d(colors)
            meshes.append(triangle_mesh)
    
    mesh = meshes.pop()
    for j in range(len(meshes)):
        mesh += meshes.pop()
    return mesh

right_mesh = image_mesh(right_image_grid, right_image_colors)
left_mesh = image_mesh(left_image_grid, left_image_colors)

def warp(points, O_lines, O_plane, n):
    u = (points - O_lines) / lalg.norm(points - O_lines, axis = 1)[:, None]
    t = ((O_plane - O_lines) @ n) / (u @ n)
    return O_lines + u * t[:, None]

warped_left_grid = warp(transformed_left_grid, left_O, O, rotation[:, 2])
warped_right_grid = warp(transformed_right_grid, right_O, O, rotation[:, 2])
warped_left_corners = warp(left_corners, left_O, O, rotation[:, 2])
warped_right_corners = warp(right_corners, right_O, O, rotation[:, 2])

def reproject(points, rotation_matrix, translation_vector):
    reprojected_grid = cv2.projectPoints(world_grid, cv2.Rodrigues(rotation_matrix)[0], translation_vector,
                                         camera_matrix, np.array([]))[0][:, 0, :]
    return reprojected_grid

reprojected_left_world_grid = reproject(world_grid, left_rotation, left_translation)
reprojected_right_world_grid = reproject(world_grid, right_rotation, right_translation)
# axes_1[1, 0].scatter(reprojected_left_world_grid[:, 0], reprojected_left_world_grid[:, 1], color = black, zorder = 3)
# axes_1[1, 1].scatter(reprojected_right_world_grid[:, 0], reprojected_right_world_grid[:, 1], color = black, zorder = 3)
# axes_1[1, 0].legend(['Image Checkerboard', 'Reprojected World Checkerboard'])
# axes_1[1, 1].legend(['Image Checkerboard', 'Reprojected World Checkerboard'])

dewarped_reprojection_errors = [lalg.norm(i - j, axis = 1) for i, j in zip([reprojected_left_world_grid, reprojected_right_world_grid],
                                                                           [left_grid.reshape((-1, 2)), right_grid.reshape((-1, 2))])]
print('Mean Dewarped Reprojection Error:')
print(np.mean(dewarped_reprojection_errors))

reprojected_left_world_grid = reprojected_left_world_grid.reshape(pattern_shape + tuple([2]))
transformed_left_world_grid = transform_grid(reprojected_left_world_grid, left_rotation, left_translation)
reprojected_right_world_grid = reprojected_right_world_grid.reshape(pattern_shape + tuple([2]))
transformed_right_world_grid = transform_grid(reprojected_right_world_grid, right_rotation, right_translation)
warped_left_world_grid = warp(transformed_left_world_grid, left_O, O, rotation[:, 2])
warped_right_world_grid = warp(transformed_right_world_grid, right_O, O, rotation[:, 2])
warped_left_image_grid = warp(left_image_grid.reshape((-1, 3)), left_O, O, rotation[:, 2]).reshape(right_image_grid.shape)
warped_right_image_grid = warp(right_image_grid.reshape((-1, 3)), right_O, O, rotation[:, 2]).reshape(left_image_grid.shape)
rectified_left_mesh = image_mesh(warped_left_image_grid, left_image_colors)
rectified_right_mesh = image_mesh(warped_right_image_grid, right_image_colors)

def vanishing_lines(translation, rotation):
    points = np.concatenate((world_grid, [-translation @ rotation]), axis = 0)
    edges = np.arange(0, len(points) - 1)[:, None][:, [0, 0]]
    edges[:, 1] = len(edges)
    lines = o3dls(v3d(points), v2i(edges))
    lines.paint_uniform_color(gray * .678)
    return lines

def corner_lines(points):
    edges = np.array([[0, 1], [1, 2], [2, 3], [3, 0]])
    lines = o3dls(v3d(points), v2i(edges))
    return lines

def cloud(points, color):
    point_cloud = o3dp(v3d(points))
    point_cloud.paint_uniform_color(color)
    return point_cloud

left_lines = vanishing_lines(left_translation, left_rotation)
right_lines = vanishing_lines(right_translation, right_rotation)

left_cloud = cloud(transformed_left_grid, blue)
right_cloud = cloud(transformed_right_grid, red)

warped_left_cloud = cloud(warped_left_grid, blue)
warped_right_cloud = cloud(warped_right_grid, red)

world_cloud = cloud(world_grid, black)

right_corner_cloud = cloud(right_corners, red)
left_corner_cloud = cloud(left_corners, blue)
right_corner_lines = corner_lines(right_corners)
left_corner_lines = corner_lines(left_corners)

warped_right_corner_cloud = cloud(warped_right_corners, red)
warped_left_corner_cloud = cloud(warped_left_corners, blue)
warped_left_corner_lines = corner_lines(warped_left_corners)
warped_right_corner_lines = corner_lines(warped_right_corners)

right_O_cloud = cloud([right_O], red)
left_O_cloud = cloud([left_O], blue)

dg(world_meshes + world_lines + [right_mesh, left_mesh, left_lines, left_cloud, right_lines, right_cloud,
                                 left_corner_cloud, right_corner_cloud, left_corner_lines,
                                 right_corner_lines, right_O_cloud, left_O_cloud],
   mesh_show_back_face = True)

dg(world_meshes + world_lines + [rectified_left_mesh, rectified_right_mesh, left_lines, warped_left_cloud, right_lines, warped_right_cloud,
                                 warped_left_corner_cloud, warped_right_corner_cloud,
                                 warped_left_corner_lines, warped_right_corner_lines,
                                 right_O_cloud, left_O_cloud], mesh_show_back_face = True)

rotated_left_corners = -(warped_left_corners @ rotation)[:, [0, 1]]
rotated_right_corners = -(warped_right_corners @ rotation)[:, [0, 1]]
rotated_left_grid = -(warped_left_grid @ rotation)[:, [0, 1]]
rotated_right_grid = -(warped_right_grid @ rotation)[:, [0, 1]]
warped_left_world_grid = -(warped_left_world_grid @ rotation)[:, [0, 1]]
warped_right_world_grid = -(warped_right_world_grid @ rotation)[:, [0, 1]]

# plt.figure('rotated corners')
# plt.scatter(rotated_left_corners[:, 0], rotated_left_corners[:, 1], color = 'blue')
# plt.scatter(rotated_left_grid[:, 0], rotated_left_grid[:, 1], color = 'blue')
# plt.scatter(rotated_right_corners[:, 0], rotated_right_corners[:, 1], color = 'red')
# plt.scatter(rotated_right_grid[:, 0], rotated_right_grid[:, 1], color = 'red')
# plt.axis('equal')

def rectify(corners, grid, image, y0):
    grid -= np.min(corners, axis = 0)
    corners -= np.min(corners, axis = 0)
    grid[:, 1] += y0
    corners[:, 1] += y0
    perspective_transform = cv2.getPerspectiveTransform(image_corners.reshape((-1, 2)),
                                                        corners.astype('float32'))
    dx = np.max(corners[:, 0]) - np.min(corners[:, 0])
    dy = np.max(corners[:, 1]) - np.min(corners[:, 1])
    target_shape = tuple([int(dx), int(dy)])
    rectified_image = cv2.warpPerspective(image, perspective_transform,
                                          target_shape, borderValue = np.nan)
    return corners, grid, rectified_image

dy = np.min(rotated_left_corners[:, 1]) - np.min(rotated_right_corners[:, 1])
if dy < 0:
    y0_left, y0_right = 0, abs(dy)
else:
    y0_left, y0_right = abs(dy), 0

_, warped_left_world_grid, _ = rectify(rotated_left_corners.copy(), warped_left_world_grid, remapped_left, y0_left)
_, warped_right_world_grid, _ = rectify(rotated_right_corners.copy(), warped_right_world_grid, remapped_right, y0_right)

rotated_left_corners, rotated_left_grid, rectified_left = rectify(rotated_left_corners, rotated_left_grid, remapped_left, y0_left)
rotated_right_corners, rotated_right_grid, rectified_right = rectify(rotated_right_corners, rotated_right_grid, remapped_right, y0_right)

rectified_reprojection_errors = [lalg.norm(i - j, axis = 1) for i, j in zip([rotated_left_grid, rotated_right_grid],
                                                                            [warped_left_world_grid, warped_right_world_grid])]
print('Mean Rectified Reprojection Error:')
print(np.mean(rectified_reprojection_errors))

images = [rectified_left, rectified_right]
grids = [rotated_left_grid, rotated_right_grid]
reprojected_grids = [warped_left_world_grid, warped_right_world_grid]
corners = [rotated_left_corners, rotated_right_corners]
colors = ['dodgerblue', red]
for i in [0, 1]:
    axes_2[i].imshow(images[i], cmap = 'gray')
    axes_2[i].plot(corners[i][[0, 1, 2, 3, 0], 0],
                   corners[i][[0, 1, 2, 3, 0], 1], '-o', color = colors[i], linewidth = width)
    
    grid = grids[i].reshape(pattern_shape + tuple([2]))
    draw_lines(grid, axes_2[i], [blue, red][i])
    # axes_2[i].scatter(reprojected_grids[i][:, 0], reprojected_grids[i][:, 1], color = black, zorder = 3)
    # axes_2[i].legend(['Rectified Image Boundary',
    #                   'Rectified Image Checkerboard',
    #                   'Reprojected World Checkerboard'])
    
combined_shape = (max([rectified_left.shape[0], rectified_right.shape[0]]),
                  max([rectified_left.shape[1], rectified_right.shape[1]]))

def resize(image):
    combined_template = np.zeros((combined_shape[0], image.shape[1])).astype('float32') + np.inf
    combined_template[0 : image.shape[0], :] = image
    return combined_template

rectified_left = resize(rectified_left)
rectified_right = resize(rectified_right)

bottom_corners = np.concatenate((rotated_left_corners[[0, 1]],
                                 rotated_right_corners[[0, 1]]))

top_corners = np.concatenate((rotated_left_corners[[2, 3]],
                              rotated_right_corners[[2, 3]]))

min_y = int(np.max(bottom_corners[:, 1]))
max_y = int(np.min(top_corners[:, 1]))
axes_3.set_ylim(max_y, min_y)

rectified_left[0 : min_y, :] = np.inf
rectified_right[0 : min_y, :] = np.inf
rectified_left[max_y ::, :] = np.inf
rectified_right[max_y ::, :] = np.inf

axes_3.imshow(np.concatenate((rectified_left, rectified_right), axis = 1), cmap = 'gray')

rotated_right_grid[:, 0] += rectified_left.shape[1]
e = []
for (p1, p2) in zip(rotated_left_grid, rotated_right_grid):
    plt.plot([p1[0], p2[0]], [p1[1], p1[1]], color = 'dodgerblue')
    plt.plot([p1[0], p2[0]], [p2[1], p2[1]], color = 'red')
    e.append(abs(p1[1] - p2[1]))
    
print('Mean Left-Right Image Alignment Error')
print(np.mean(e))
