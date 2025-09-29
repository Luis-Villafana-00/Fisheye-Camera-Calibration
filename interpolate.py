import cv2, numpy as np, matplotlib.pyplot as plt
import utilities as ut, json

n_interp = 5
arc_ratio = .1
width = 3.45 / 1.345
image_alpha = .234#.345
mesh_alpha = .789
size = 7
blue = np.array([0, 137, 250]) / 255
red = np.array([255, 43, 135]) / 255
black = red * 0
pattern_size = (7, 6)
n_refined = 10 ** 5

'''load data and detect corners'''
image_path = 'images/4k fisheye/for dewarping/'
image_names = ['0.jpg', '1.jpg', '2.jpg', '3.jpg', '4.jpg', '5.jpg']
mask_names = {'1.jpg' : '1 mask.png', '2.jpg' : '2 mask.png',
              '3.jpg' : '3 mask.png', '4.jpg' : '4 mask.png'}

'''uncomment if optical center has been obtained'''
parameters_path = 'camera parameters/2025/'
try:
    with open(parameters_path + 'parameters.json', 'r') as file:
        parameters = json.load(file)
        parameters_file = True
        O = np.array(parameters['O1'])
except:
    parameters_file = False

for i, image_name in enumerate(image_names):
    print(i)
    loaded_image = 255 - cv2.imread(image_path + image_name, 0)
    figure, axes = plt.subplots(nrows = 2, ncols = 2)
    for (j, k) in zip([0, 0, 1, 1], [0, 1, 0, 1]):
        axes[j, k].set_xticks([])
        axes[j, k].set_yticks([])
        axes[j, k].imshow(loaded_image, cmap = 'gray', alpha = image_alpha)
        
    plt.show()
    
    '''color of mask: {36, 28, 237}'''
    mask = None
    if image_name in mask_names:
        mask_name = mask_names[image_name]
        mask = cv2.imread(image_path + mask_name)
        mask = (mask[:, :, 0] == 36) * (mask[:, :, 1] == 28) * (mask[:, :, 2] == 237)
    
    grid = ut.detect_corners(loaded_image, pattern_size = pattern_size, mask = mask)
    grid = np.array(grid).reshape(pattern_size[:: -1] + tuple([2])).transpose((1, 0 ,2))
    grid = ut.interpolate_grid(grid, n_interp, pattern_size)
    if parameters_file:
        critical_points_list = []
        for j in [0, 1]:
            critical_points, grid = ut.find_critical_points(grid, O, n_refined)
            grid = grid.transpose((1, 0, 2))
            critical_points_list.append(critical_points)
    
    for j in [0, 1]:
        for k in range(0, len(grid), n_interp):
            axes[0, 0].plot(grid[k, :, 0], grid[k, :, 1], linewidth = width, color = [red, red][j])
        grid = grid.transpose((1, 0, 2))
        
    for j in [0, 1]:
        arc_indices = np.linspace(1, len(grid), int(len(grid) * arc_ratio)).astype('int') - 1
        for k in range(len(grid)):#range(len(grid)):#arc_indices:
            axes[0, 1].plot(grid[k, :, 0], grid[k, :, 1], linewidth = width, color = [blue, red][j])
        grid = grid.transpose((1, 0, 2))

    for j in [0, 1]:
        arc_indices = np.linspace(1, len(grid), int(len(grid) * arc_ratio)).astype('int') - 1
        for k in range(len(grid)):#range(len(grid)):#arc_indices:
            axes[1, j].plot(grid[k, :, 0], grid[k, :, 1], linewidth = width, color = [blue, red][j])
        grid = grid.transpose((1, 0, 2))
        if parameters_file:
            axes[1, j].scatter([O[0]], [O[1]], color = 'white', s = 2.222 * size, zorder = 4)
            axes[1, j].plot(critical_points_list[j][:, 0], critical_points_list[j][:, 1], linewidth = width, color = black, zorder = 3)
