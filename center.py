import cv2, numpy as np, matplotlib.pyplot as plt, os, numpy.linalg as lalg
import utilities as ut, json
from scipy.interpolate import griddata as gd

n_interp = 456
n_grid = 234
n_deflections = 17
width = 3.45
image_alpha = .234
mesh_alpha = .789
size = 55
blue = np.array([0, 137, 250]) / 255
red = np.array([255, 43, 135]) / 255
black = red * 0
pattern_size = (7, 6)

def plot_deflections(image_x, image_y, grid, summed_deflections, axis, plotting = True):
    indices_1 = np.linspace(0, grid.shape[0] - 1, n_grid).astype('int')
    indices_2 = np.linspace(0, grid.shape[1] - 1, n_grid).astype('int')
    grid = grid[indices_1][:, indices_2, :]
    z = summed_deflections[indices_1][:, None].repeat(len(indices_2), axis = 1)
    z = z.flatten()
    grid = grid.reshape((-1, 2))
    grid_z = gd((grid[:, 0], grid[:, 1]), z, (image_x, image_y), method = 'cubic')
    if plotting:
        axis.imshow(grid_z, alpha = mesh_alpha, cmap = 'rainbow')
    return grid_z

'''load data and detect corners'''
image_path = 'images/4k fisheye/for center/'
names = os.listdir(image_path)
image_names = ['0.jpg', '1.jpg', '2.jpg', '3.jpg', '4.jpg']
mask_names = {'1.jpg' : '1 mask.png', '2.jpg' : '2 mask.png',
              '3.jpg' : '3 mask.png', '4.jpg' : '4 mask.png'}

if len(image_names) > 1:
    n_cols = 2
n_rows = int(len(image_names) / 2) + len(image_names) % 2
figure_1, axes_1 = plt.subplots(nrows = n_rows, ncols = n_cols)
if len(image_names) % 2 and len(image_names) > 2:
    axes_1[-1, -1].remove()
for j in axes_1.flat:
    j.set_xticks([])
    j.set_yticks([])

parameters_path = 'camera parameters/2025/'
central_arcs = []
all_deflections = []
loaded_images = []
parameters = {}
for i, image_name in enumerate(image_names):
    #plt.close('all')
    print(i)
    loaded_image = 255 - cv2.imread(image_path + image_name, 0)
    image_y, image_x = np.mgrid[0 : loaded_image.shape[0], 0 : loaded_image.shape[1]]
    loaded_images.append(loaded_image)
    
    figure_2, axes_2 = plt.subplots(nrows = 2, ncols = 2)
    for (j, k) in zip([0, 0, 1, 1], [0, 1, 0, 1]):
        axes_2[j, k].set_xticks([])
        axes_2[j, k].set_yticks([])
        axes_2[j, k].imshow(loaded_image, cmap = 'gray', alpha = image_alpha)
    plt.show()
    
    '''color of mask: {36, 28, 237}'''
    mask = None
    if image_name in mask_names:
        mask_name = mask_names[image_name]
        mask = cv2.imread(image_path + mask_name)
        mask = (mask[:, :, 0] == 36) * (mask[:, :, 1] == 28) * (mask[:, :, 2] == 237)
    
    grid = ut.detect_corners(loaded_image, pattern_size = pattern_size, mask = mask)
    grid = np.array(grid).reshape(pattern_size[:: -1] + tuple([2])).transpose((1, 0 ,2))
    
    '''create cubic interpolations of loaded grid'''
    grid = ut.interpolate_grid(grid, n_interp, pattern_size)

    '''find and plot central arcs'''
    deflection_plots = []
    for k in [0, 1]:
        p1 = grid[:, 0, :]
        p2 = grid[:, -1, :]
        u = p2 - p1
        u /= lalg.norm(u, axis = 1)[:, None]
        v = grid - p1[:, None, :]
        projections = p1[:, None, :] + u[:, None, :] * np.sum(u[:, None, :] * v, axis = 2)[:, :, None]
        deflections = lalg.norm(grid - projections,axis = 2)
        summed_deflections = np.sum(deflections, axis = 1)
        summed_deflections -= np.min(summed_deflections)
        summed_deflections /= np.max(summed_deflections)
        deflections_argmin = np.argmin(summed_deflections)
        central_arc = np.array([p1[deflections_argmin], p2[deflections_argmin]])
        central_arcs.append(central_arc)
        arc_indices = np.arange(0, len(grid), n_interp).astype('int')

        for j in arc_indices:
            arc = grid[j]
            axes_2[0, k].plot(arc[:, 0], arc[:, 1], color = blue, linewidth = width)
            
            axes_2[0, k].plot([p1[j, 0], p2[j, 0]], [p1[j, 1], p2[j, 1]], color = black, linewidth = width)
            deflection_indices = np.linspace(0, grid.shape[1] - 1, n_deflections).astype('int')
            for l in deflection_indices:
                axes_2[0, k].plot([projections[j, l, 0], grid[j, l, 0]],
                                  [projections[j, l, 1], grid[j, l, 1]], color = black, linewidth = width)

        deflection_plots.append(plot_deflections(image_x, image_y, grid,
                                                 summed_deflections, axes_2[1, k]))
        
        axes_2[1, k].plot(central_arc[:, 0], central_arc[:, 1], color = black, linewidth = width)
        grid = grid.transpose((1, 0, 2))

    combined_deflections = np.max(deflection_plots,axis = 0)
    all_deflections.append(combined_deflections)
    axes_1.flat[i].imshow(loaded_image, cmap = 'gray', alpha = image_alpha)
    axes_1.flat[i].imshow(combined_deflections, cmap = 'rainbow', alpha = mesh_alpha)
    for j in [-1, -2]:
        axes_1.flat[i].plot(central_arcs[j][:, 0], central_arcs[j][:, 1],
                            color = black, linewidth = width)

combined_deflections = np.max(all_deflections, axis = 0)
mask = combined_deflections == np.min(combined_deflections[~np.isnan(combined_deflections)])
mean_x = np.mean(image_x[mask])
mean_y = np.mean(image_y[mask])
O1 = np.array([mean_x, mean_y])

figure_3, axes_3 = plt.subplots()
axes_3.imshow(loaded_images[-1], alpha = image_alpha, cmap = 'gray')
axes_3.imshow(combined_deflections, cmap = 'rainbow', alpha = mesh_alpha)
for central_arc in central_arcs:
    axes_3.plot(central_arc[:, 0], central_arc[:, 1],
                color = black, linewidth = width / 2)
axes_3.scatter([O1[0]], [O1[1]], color = red, zorder = 4)
axes_3.set_xticks([])
axes_3.set_yticks([])

parameters['O1'] = O1.tolist()
with open(parameters_path + 'parameters.json', 'w') as file:
    json.dump(parameters, file, indent = 4)
