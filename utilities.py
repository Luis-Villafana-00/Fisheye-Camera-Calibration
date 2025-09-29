import cv2, numpy as np, numpy.linalg as lalg
from scipy.interpolate import CubicSpline as cbsp
from scipy.interpolate import RectBivariateSpline as bv
from open3d.utility import Vector3dVector as v3d
from open3d.utility import Vector3iVector as v3i
from open3d.utility import Vector2iVector as v2i
import open3d as o3d
from open3d.geometry import LineSet as o3dls

def interpolate_grid(grid, n_interp, pattern_size):
    t1 = np.arange(0, pattern_size[0])
    t1_fine = np.linspace(0, pattern_size[0], n_interp * pattern_size[0] + 1)
    t2 = np.arange(0, pattern_size[1])
    t2_fine = np.linspace(0, pattern_size[1], n_interp * pattern_size[1] + 1)
    t_to_x = bv(t1, t2, grid[:, :, 0], s = 0)
    t_to_y = bv(t1, t2, grid[:, :, 1], s = 0)
    grid_x = t_to_x(t1_fine, t2_fine)[:, :, None]
    grid_y = t_to_y(t1_fine, t2_fine)[:, :, None]
    grid = np.concatenate((grid_x, grid_y), axis = 2)[0 : -n_interp, 0 : -n_interp, :]
    return grid

def find_critical_points(grid, O, n_refined):
    critical_points = []
    for k in range(len(grid) - 1, -1, -1):
        arc = grid[k]
        d = lalg.norm(arc - O, axis = 1)
        t = np.arange(0, len(arc))
        argmin = np.argmin(d)
        if argmin == 0 or argmin == len(arc) - 1:
            grid = np.delete(grid, k, axis = 0)
            continue
        t1 = t[argmin - 1]
        t3 = t[argmin + 1]
        t_to_x = cbsp(t, arc[:, 0])
        t_to_y = cbsp(t, arc[:, 1])
        refined_t = np.linspace(t1, t3, n_refined)
        t = np.sort(np.concatenate((t, refined_t)))
        refined_arc = np.array([t_to_x(t), t_to_y(t)]).T
        d = lalg.norm(refined_arc - O, axis = 1)
        critical_points.insert(0, refined_arc[np.argmin(d)])
    return np.array(critical_points), grid

def refine_grid(grid):
    A = np.array([[0, 1], [-1, 0]])
    for j in range(grid.shape[0]):
        for k in range(grid.shape[1]):
            p1 = grid[j, 0][:, None]
            p2 = grid[j, -1][:, None]
            p3 = grid[0, k][:, None]
            p4 = grid[-1, k][:, None]
            t = (((p2 - p1).T @ A @ (p3 - p1)) / ((p4 - p3).T @ A @ (p2 - p1)))[0] 
            p = p3 + (p4 - p3) * t
            grid[j, k] = p[:, 0]
    return grid

def nearest_intersection(points, dirs):
    '''
    Taken from Stack Overflow
    :param points: (N, d) array of points on the lines
    :param dirs: (N, d) array of unit direction vectors
    :returns: (d,) array of intersection point
    '''
    dirs_mat = dirs[:, :, np.newaxis] @ dirs[:, np.newaxis, :]
    points_mat = points[:, :, np.newaxis]
    I = np.eye(points.shape[1])
    return lalg.lstsq((I - dirs_mat).sum(axis = 0),
                      ((I - dirs_mat) @ points_mat).sum(axis = 0), rcond = None)[0]

def detect_corners(image, pattern_size = (6, 7), mask = None):
    search_window = (12, 12)
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    try:
        if mask is None:
            retval, corners = cv2.findChessboardCorners(image, pattern_size)
            if retval:
                # refining pixel coordinates for given 2d points.
                corners = cv2.cornerSubPix(image, corners, search_window, (-1, -1), criteria)[:, 0, :]
        else:
            dst = cv2.cornerHarris(image, 17, 7, .04)
            dst[~mask] = 0
            ret, dst = cv2.threshold(dst, .01 * dst.max(), 255, 0)
            ret, labels, stats, centroids = cv2.connectedComponentsWithStats(dst.astype('uint8'))
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.001)
            corners = cv2.cornerSubPix(image, np.float32(centroids), (7, 7), (-1, -1), criteria)
            corners = corners[mask[corners[:, 1].astype('int'), corners[:, 0].astype('int')]]
            corners = corners[np.argsort(corners[:, 1])]
            grouped_corners, segment = [], []
            for j, corner in enumerate(corners):
                segment.append(corner)
                if len(segment) == pattern_size[0]:
                    segment = np.array(segment)
                    grouped_corners.append(segment[np.argsort(segment[:, 0])[:: -1]])
                    segment = []
            corners = np.concatenate(grouped_corners, axis = 0)
            
        return corners
    except Exception as E:
        print(E)
        return None

def binomial_coefficients(k):
    B = np.zeros((k, k))
    B[:, 0] = 1
    B[1, 1] = 1
    for j in range(2, len(B)):
        B[j, 1 ::] += B[j - 1, 0 : -1]
        B[j, 1 ::] += B[j - 1, 1 ::]

    return B[-1]

def pose(image_grid, world_grid, camera_matrix):
    _, rotation, translation = cv2.solvePnP(world_grid.reshape((-1, 3)),
                                            image_grid.reshape((-1, 2)),
                                            camera_matrix, None, flags = 0)
    
    return cv2.Rodrigues(rotation)[0], translation[:, 0]

def create_meshes(grid, color_1, color_2):
    meshes = []
    for j in range(len(grid) - 1):
        for k in range(len(grid[0]) - 1):
            p1 = grid[j, k]
            p2 = grid[j, k + 1]
            p3 = grid[j + 1, k + 1]
            p4 = grid[j + 1, k]
            color = [[color_1, color_2], [color_2, color_1]][j % 2][k % 2]
            T = np.array([[0, 1, 2], [0, 2, 3]])
            P = np.array([p1, p2, p3, p4])
            triangle_mesh = o3d.geometry.TriangleMesh()
            triangle_mesh.vertices = v3d(P)
            triangle_mesh.triangles = v3i(T)
            triangle_mesh.paint_uniform_color(color)
            triangle_mesh.compute_triangle_normals()
            meshes.append(triangle_mesh)
    return meshes

def create_lines(grid, color = None):
    line_collection = []
    for j in range(len(grid) - 1):
        for k in range(len(grid[0]) - 1):
            p1 = grid[j, k]
            p2 = grid[j, k + 1]
            p3 = grid[j + 1, k + 1]
            p4 = grid[j + 1, k]
            E = np.array([[0, 1], [1, 2], [2, 3], [0, 3]])
            P = np.array([p1, p2, p3, p4])
            lines = o3dls(v3d(P), v2i(E))
            if color is not None:
                lines.paint_uniform_color(color)
            line_collection.append(lines)
    return line_collection