# Fisheye-Camera-Calibration
An alternative approach to calibrating cameras with radially symmetric distortion with a minimal number of checkerboard images.

## Main Building Block - Parametric Rectangle Interpolation

<img width="1748" height="946" alt="image" src="https://github.com/user-attachments/assets/96f56296-77f6-4dee-963b-6a162758da0f" />

## Optical Center Estimation - Deflection Minimization

<img width="1743" height="947" alt="image" src="https://github.com/user-attachments/assets/9fbec5c6-d88d-4912-9804-4a258f345281" />

### One image is generally sufficient for finding the point of least deflection, but multiple images can be used for robustness:

<img width="1425" height="943" alt="image" src="https://github.com/user-attachments/assets/d3f7148c-af89-4091-a4d0-b0a003873118" />

### Aggregated point of least deflection:

<img width="1671" height="942" alt="image" src="https://github.com/user-attachments/assets/de252736-645f-46ce-9f3a-21a79a5d30d7" />

## Dewarping Barrel Distortion With Nonnegative Linear Least-Squares Optimization

<img width="1262" height="704" alt="image" src="https://github.com/user-attachments/assets/4de53660-ff01-4341-a618-55c58414414a" />

### Some examples of fisheye images and their dewarped counterparts:

<img width="1901" height="580" alt="image" src="https://github.com/user-attachments/assets/df0eec4d-f77e-415d-b7cf-eac5e189d043" />

<img width="1902" height="540" alt="image" src="https://github.com/user-attachments/assets/651af1ce-f651-4468-bb6e-e41907a4c3ff" />

<img width="1899" height="536" alt="image" src="https://github.com/user-attachments/assets/c3e73470-2f4e-4e66-8a9a-5a93b8faf066" />

## Estimating Focal Distance With OpenCV

