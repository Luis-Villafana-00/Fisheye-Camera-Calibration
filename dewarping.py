import cv2, numpy as np, matplotlib.pyplot as plt, os, numpy.linalg as lalg
from scipy.interpolate import CubicSpline as cbsp
import utilities as ut, json
from scipy.optimize import nnls as nnls

n_interp = 89
n_equations = 67
n_map = 111111
width = 3.45 / 1.345
image_alpha = .234
mesh_alpha = .789
arc_alpha = .555
size = 7 * 2.222
blue = np.array([0, 137, 250]) / 255
red = np.array([255, 43, 135]) / 255
black = red * 0
pattern_size = (7, 6)
N = 17
n_refined = 10 ** 5

'''load data and detect corners'''
image_path = 'images/4k fisheye/for dewarping/'
names = os.listdir(image_path)
image_names = ['0.jpg', '1.jpg', '2.jpg', '3.jpg', '4.jpg', '5.jpg']#, '6.jpg']
mask_names = {'1.jpg' : '1 mask.png', '2.jpg' : '2 mask.png',
              '3.jpg' : '3 mask.png', '4.jpg' : '4 mask.png',
              '6.jpg' : '6 mask.png'}

parameters_path = 'camera parameters/2025/'
central_arcs = []
all_deflections = []
loaded_images = []
with open(parameters_path + 'parameters.json', 'r') as file:
    parameters = json.load(file)

O1 = np.array(parameters['O1'])
p1, p2, v1, v2 = [], [], [], []
loaded_grids = []
figures, axes_list = [], []
for i, image_name in enumerate(image_names):
    #plt.close('all')
    print(i)
    loaded_image = 255 - cv2.imread(image_path + image_name, 0)
    image_y, image_x = np.mgrid[0 : loaded_image.shape[0], 0 : loaded_image.shape[1]]
    loaded_images.append(loaded_image)

    figure, axes = plt.subplots(nrows = 1, ncols = 2)
    figures.append(figure)
    axes_list.append(axes)
    for j in [0, 1]:
        axes[j].set_xticks([])
        axes[j].set_yticks([])
    plt.show()
    
    '''36, 28, 237'''
    mask = None
    if image_name in mask_names:
        mask_name = mask_names[image_name]
        mask = cv2.imread(image_path + mask_name)
        mask = (mask[:, :, 0] == 36) * (mask[:, :, 1] == 28) * (mask[:, :, 2] == 237)
    
    grid = ut.detect_corners(loaded_image, pattern_size = pattern_size, mask = mask)
    grid = np.array(grid).reshape(pattern_size[:: -1] + tuple([2])).transpose((1, 0 ,2))
    loaded_grids.append(grid)
    
    '''create cubic interpolations of loaded grid'''
    grid = ut.interpolate_grid(grid, n_interp, pattern_size)

    axes[0].imshow(loaded_image, cmap = 'gray', alpha = image_alpha)
    for j in [0, 1]:
        for k in range(0, len(grid), n_interp):
            axes[0].plot(grid[k, :, 0], grid[k, :, 1], linewidth = width, color = red, alpha = mesh_alpha)
        grid = grid.transpose((1, 0, 2))
        
    '''find critical points and transform arc endpoints'''
    for j in [0, 1]:
        critical_points, filtered_grid = ut.find_critical_points(grid, O1, n_refined)
        
        '''gather data for system of equations'''
        if len(filtered_grid) > 0:
            equation_indices = np.linspace(0, filtered_grid.shape[1] - 1, n_equations).astype('int')
            p1.append(critical_points[:, None, :].repeat(len(equation_indices), axis = 1).reshape((-1, 2)))
            p2.append(filtered_grid[:, equation_indices, :].reshape(-1, 2))
        
        grid = grid.transpose((1, 0, 2))

p1, p2 = [np.concatenate(j, axis = 0) for j in [p1, p2]]

'''create system of equations'''
v1, v2 = [(j - O1) / lalg.norm(j - O1, axis = 1)[:, None] for j in [p1, p2]]
r1, r2 = [lalg.norm(j - O1, axis = 1) for j in [p1, p2]]
rho_1 = np.max([r1, r2])
r1, r2 = [j / rho_1 for j in [r1, r2]]

n = np.arange(0, N)
R1, R2 = [(j[:, None] ** n) * ((1 - j[:, None]) ** (N -1 - n)) for j in [r1, r2]]
B = np.diag(ut.binomial_coefficients(N))
B[[0, 1], [0, 1]] = [0, 0]

M = (((v1 * v2) @ [1, 1])[:, None] * R2 - R1) @ B
b = (v1 * (p1 - p2)) @ [1, 1]

'''solve for control points'''
K = nnls(M, b)[0]

warped_radii = np.linspace(0, rho_1, n_map)
normalized_radii = warped_radii / rho_1
R = (normalized_radii[:, None] ** n) * ((1 - normalized_radii[:, None]) ** (N -1 - n))
distortions = R @ B @ K[:, None]
dewarped_radii = warped_radii + distortions[:, 0]
rho_2 = np.max(dewarped_radii)
warped_image_radii = ((image_x - O1[0]) ** 2 + (image_y - O1[1]) ** 2) ** .5
warped_image_radii[warped_image_radii > rho_1] = np.nan

_, axes = plt.subplots()
axes.plot(warped_radii, dewarped_radii)
axes.minorticks_on()
axes.grid(which = 'both')
axes.set_xlim(left = 0, right = rho_1 * 1.05)
axes.set_aspect('equal')

dewarp = cbsp(warped_radii, dewarped_radii)
rewarp = cbsp(dewarped_radii, warped_radii)
dewarped_image_radii = dewarp(warped_image_radii)
x_axis = dewarped_image_radii[int(O1[1]), :]
y_axis = dewarped_image_radii[:, int(O1[0])]
x_axis[image_x[0, :] < O1[0]] *= -1
y_axis[image_y[:, 0] < O1[1]] *= -1
dewarped_width = np.max(x_axis[~np.isnan(x_axis)]) - np.min(x_axis[~np.isnan(x_axis)])
dewarped_height = np.max(y_axis[~np.isnan(y_axis)]) - np.min(y_axis[~np.isnan(y_axis)])
O2 = np.array([0 - np.min(x_axis[~np.isnan(x_axis)]),
               0 - np.min(y_axis[~np.isnan(y_axis)])])

dewarped_image_y, dewarped_image_x = np.mgrid[0 : dewarped_height, 0 : dewarped_width]
dewarped_image_x = dewarped_image_x - O2[0]
dewarped_image_y = dewarped_image_y - O2[1]
dewarped_image_radii = ((dewarped_image_x) ** 2 + (dewarped_image_y) ** 2) ** .5
dewarped_image_radii[dewarped_image_radii > rho_2] = np.nan
rewarped_image_radii = rewarp(dewarped_image_radii)
cosines = dewarped_image_x / dewarped_image_radii
sines = dewarped_image_y / dewarped_image_radii
map_x = (cosines * rewarped_image_radii).astype('float32') + O1[0].astype('float32')
map_y = (sines * rewarped_image_radii).astype('float32') + O1[1].astype('float32')

np.save(parameters_path + 'map x.npy', map_x)
np.save(parameters_path + 'map y.npy', map_y)
parameters['O2'] = O2.tolist()
parameters['rho 1'] = rho_1
parameters['rho 2'] = rho_2
parameters['K'] = K.tolist()
with open(parameters_path + 'parameters.json', 'w') as file:
    json.dump(parameters, file, indent = 4)

def dewarp_points(p):
    r = lalg.norm(p, axis = 1)
    v = p / r[:, None]
    r = dewarp(r)
    p = v * r[:, None]
    return p

'''remap images and dewarp corners'''
for j, loaded_grid in enumerate(loaded_grids):
    remapped_image = cv2.remap(loaded_images[j].astype('float32'), map_x, map_y,
                               interpolation = cv2.INTER_LINEAR, borderValue = np.inf)
    interpolated_grid = ut.interpolate_grid(loaded_grid, n_interp, pattern_size)
    p = loaded_grid.reshape((-1, 2)) - O1
    p = dewarp_points(p) + O2
    p = p.reshape(loaded_grid.shape)
    p = ut.refine_grid(p)
    axes_list[j][1].imshow(remapped_image, cmap = 'gray', alpha = image_alpha)
    for l in [0, 1]:
        for k in range(p.shape[0]):
            axes_list[j][1].plot([p[k, 0, 0], p[k, -1, 0]], [p[k, 0, 1], p[k, -1, 1]],
                                 color = red, linewidth = width, alpha = mesh_alpha)
        p = p.transpose((1, 0, 2))
    
    '''This was for the Readme.md'''
    # p = p - O2[None, None, :] + O1[None, None, :]
    # critical_points = ut.find_critical_points(interpolated_grid, O1, n_refined)[0]
    # dewarped_critical_points = dewarp_points(critical_points - O1) + O1
    # _, axes = plt.subplots()
    # axes.imshow(loaded_images[j], cmap = 'gray', alpha = image_alpha)
    # axes.plot(interpolated_grid[0, :, 0], interpolated_grid[0, :, 1], color = red, linewidth = width)
    # axes.plot([p[0, 0, 0], p[0, -1, 0]], [p[0, 0, 1], p[0, -1, 1]], color = blue,
    #           linewidth = width)
    # axes.scatter([O1[0]], [O1[1]], color = black, s = size, zorder = 4)
    # axes.plot([O1[0], p[0, 0, 0]], [O1[1], p[0, 0, 1]], color = black)
    # axes.plot([O1[0], p[0, -1, 0]], [O1[1], p[0, -1, 1]], color = black)
    # axes.plot([O1[0], dewarped_critical_points[0, 0]],
    #           [O1[1], dewarped_critical_points[0, 1]], color = black)
    # axes.scatter([critical_points[0, 0]], [critical_points[0, 1]],
    #              color = black, s = size, zorder = 12)
    # axes.scatter([dewarped_critical_points[0, 0]], [dewarped_critical_points[0, 1]],
    #              color = black, s = size, zorder = 12)
    # axes.scatter(interpolated_grid[0, [0, -1], 0], interpolated_grid[0, [0, -1], 1],
    #              color = red, s = size, zorder = 12)
    # axes.scatter(p[0, [0, -1], 0], p[0, [0, -1], 1], color = blue, s = size, zorder = 12)
    # axes.plot([p[0, 0, 0], p[0, -1, 0]], [p[0, 0, 1], p[0, -1, 1]], color = blue,
    #           linewidth = width)
