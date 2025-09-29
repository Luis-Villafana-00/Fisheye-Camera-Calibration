# Fisheye-Camera-Calibration
An alternative approach to calibrating stereo cameras with radially symmetric distortion with a minimal number of checkerboard images.

Example images can be found [here](https://drive.google.com/drive/folders/1ogV-BtqnnRtYcgh7X9OszqQUq7eGsr02?usp=sharing).

The shared scripts demonstrate the proposed calibration approach in the steps described below:

## Main Building Block - Parametric Rectangle Interpolation (interpolate.py)

<img width="1748" height="946" alt="image" src="https://github.com/user-attachments/assets/96f56296-77f6-4dee-963b-6a162758da0f" />

## Optical Center Estimation - Deflection Minimization (center.py)

<img width="1743" height="947" alt="image" src="https://github.com/user-attachments/assets/9fbec5c6-d88d-4912-9804-4a258f345281" />

### One image is generally sufficient for finding the point of least deflection, but multiple images can be used for robustness:

<img width="1425" height="943" alt="image" src="https://github.com/user-attachments/assets/d3f7148c-af89-4091-a4d0-b0a003873118" />

### Aggregated point of least deflection:

<img width="1671" height="942" alt="image" src="https://github.com/user-attachments/assets/de252736-645f-46ce-9f3a-21a79a5d30d7" />

## Dewarping Barrel Distortion With Nonnegative Linear Least-Squares Optimization (dewarping.py)

<img width="1374" height="775" alt="image" src="https://github.com/user-attachments/assets/be47e45f-11ad-41e2-9902-7a2f02315b56" />

### Some examples of fisheye images and their dewarped counterparts:

<img width="1901" height="580" alt="image" src="https://github.com/user-attachments/assets/df0eec4d-f77e-415d-b7cf-eac5e189d043" />

<img width="1902" height="540" alt="image" src="https://github.com/user-attachments/assets/651af1ce-f651-4468-bb6e-e41907a4c3ff" />

<img width="1899" height="536" alt="image" src="https://github.com/user-attachments/assets/c3e73470-2f4e-4e66-8a9a-5a93b8faf066" />

## Estimating Focal Distance With OpenCV (focal distance.py)

<kbd>
<img width="909" height="796" alt="image" src="https://github.com/user-attachments/assets/c6681754-2bb6-440e-8819-327578b42907" />
</kbd>

<img width="1553" height="942" alt="image" src="https://github.com/user-attachments/assets/18c3e4b3-8fde-4241-bc38-23db8225426a" />

## Stereo Camera Calibration (stereo calibration.py)

### Dewarping of Left and Right Images

<img width="1904" height="932" alt="image" src="https://github.com/user-attachments/assets/47af7c02-4c7c-42aa-90d9-d81ca499a6a8" />

### Image Rectification

<img width="1906" height="534" alt="image" src="https://github.com/user-attachments/assets/d30634ae-8831-4a02-8c01-d3a275a5db3e" />

<img width="1900" height="278" alt="image" src="https://github.com/user-attachments/assets/8a207f4b-3021-40be-a9f5-fc9936dcead7" />

<img width="1901" height="380" alt="image" src="https://github.com/user-attachments/assets/a99f20d6-8655-478f-a167-82b67a454955" />

<img width="953" height="725" alt="image" src="https://github.com/user-attachments/assets/08967a54-b922-4b27-b512-a467b10932b0" />

### Geometric Interpretation of Stereoscopic Rectification

#### Before Rectification (Images Are Skew):

<kbd>
<img width="1242" height="479" alt="image" src="https://github.com/user-attachments/assets/dd37f934-cd80-47a1-a6b8-822fb5e0f7d2" />
</kbd>

<kbd>
<img width="1272" height="495" alt="image" src="https://github.com/user-attachments/assets/96489eec-d639-4d7e-b27b-f118b0221e7a" />
</kbd>

#### After Rectification (Images Are Parallel):

<kbd>
<img width="973" height="372" alt="image" src="https://github.com/user-attachments/assets/b053adb5-2e09-454a-abde-bf884096ca4a" />
</kbd>

<kbd>
<img width="1238" height="371" alt="image" src="https://github.com/user-attachments/assets/cbce40fa-4908-49d7-8edd-f3bde8b45459" />
</kbd>
