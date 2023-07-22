# Robotics-Internship-2022
Included PID Control on Encoder Motors, Servo Motors, Forward and Inverse Kinematics. Internship with Mr. Eric Busboom in the summer of 2022

# **Advanced Lane Finding** 


The goals / steps of this project are the following:

* Compute the camera calibration matrix and distortion coefficients given a set of chessboard images.
* Apply a distortion correction to raw images.
* Use color transforms, gradients, etc., to create a thresholded binary image.
* Apply a perspective transform to rectify binary image ("birds-eye view").
* Detect lane pixels and fit to find the lane boundary.
* Determine the curvature of the lane and vehicle position with respect to center.
* Warp the detected lane boundaries back onto the original image.
* Output visual display of the lane boundaries and numerical estimation of lane curvature and vehicle position.

I demonstrate these steps of the pipeline on calibration and test images.

All of the code is contained in the IPython notebook AdvancedLaneFinding.ipynb


## 1. One Time Steps

There are a few steps that are required to be done one time before we process any images - camera calibration is one of them


### a. Camera Calibration

#### i. Camera matrix and distortion coefficients

We start by preparing "object points", which will be the (x, y, z) coordinates of the chessboard corners in the world. Here I am assuming the chessboard is fixed on the (x, y) plane at z=0, such that the object points are the same for each calibration image.  Thus, `objp` is just a replicated array of coordinates, and `objpoints` will be appended with a copy of it every time we successfully detect all chessboard corners in a test image.  `imgpoints` will be appended with the (x, y) pixel position of each of the corners in the image plane with each successful chessboard detection.  

The chess board corners are demonstrated in the image below:
<img src="./output_images/ChessCorners_calibration2.jpg" width="480" alt="Chess Corners Images" />

I then used the output `objpoints` and `imgpoints` to compute the camera calibration and distortion coefficients using the `cv2.calibrateCamera()` function.  I applied this distortion correction to the test image using the `cv2.undistort()` function and obtained this result: 

<img src="./output_images/UndistortedChess_calibration2.jpg" width="480" alt="Undistorted Chess Image" />

### b. Line Class, Perspective transform, Pixel to World Space Global variable Definitions

I used the `Line` class to hold various data across frames for the image processing for vidoes

All of the images for the pipeline are assumed to be of the same size and shape of the image is a global variable. Similarly `src` quadrilateral and `dst` rectangles for perspective transform and the pixel to world space paramaters are global variables.

#### i. M, Minv matrices for perspective transform

First we undistorted one of the straight line images amd looked at a `src` region of interest shown in image below:

<img src="./output_images/Undistorted_SrcRect_straight_lines2.jpg" width="480" alt="Undistorted Straight Line Image with Src Lines" />

Based on this I define the src and dst
```python
src = np.float32([[192,imshape[0]],[imshape[1]//2 - 55, imshape[0]//2+95], [imshape[1]//2+57, imshape[0]//2+95], [imshape[1]-162,imshape[0]]])
offset = 300 # offset for dst points
dst = np.float32([[offset, imshape[0]],[offset, 0], [imshape[1]-offset, 0], 
                  [imshape[1]-offset, imshape[0]], 
                  ])
```
This resulted in the following source and destination points:
```python
Source
[[  192.   720.]
 [  585.   455.]
 [  697.   455.]
 [ 1118.   720.]]
Destination
[[ 300.  720.]
 [ 300.    0.]
 [ 980.    0.]
 [ 980.  720.]]
M = cv2.getPerspectiveTransform(src, dst)
``` 

From this I computed the perspective transform matrices `M`, `Minv`. Warping the straight line image shows that the straight lines are parallel to the dest rectangle

<img src="./output_images/UndistortedWarped_straight_lines2.jpg" width="480" alt="Warped Straight Line Image with Dst Lines" />

#### ii. Pixel Space to World Space

From the straight line image it is evident that the width of the lane is about 650 pixels which represents a 3.7m wide line. The length of the image is about 8 dashed line segments. Using 3m per dashed line segements I chose a value of 25m

So, with this I decided on the following parameters

```python
# Define conversions in x and y from pixels space to meters
ym_per_pix = 25/720 # meters per pixel in y dimension
xm_per_pix = 3.7/650 # meters per pixel in x dimension
```

### Pipeline (single images)

These are the steps in the pipelines. They are individually tested and visualized in the python notebook. Below are the results on test2.jpg

#### 1. Undistort the image

Using the `mtx` and `dst` from camera calibration with:

    `undistorted = cv2.undistort(image, mtx, dist, None, None)`
    
With the src region displayed, the test image looks like:

<img src="./output_images/Undistorted_SrcRect_test2.jpg" width="480" alt="Undistorted test2  Image with Src Lines" />


#### 2. Warp the undistorted image

Using the M matrix to transform the src region to dst  rectangle 

    `warped = cv2.warpPerspective(undistorted , M, imshape[1::-1])`

we get the following image:

<img src="./output_images/UndistortedWarped_test2.jpg" width="480" alt="Undistorted Warped  test2 Image with Dst Lines" />

    
#### 3.  Apply thresholding to warped image - for white and yellow colors

Next we apply thresholding on the warped image:
    `binary_warped, yellow, white = thresholding_final(warped)`
    
I tried a lot of different thresholds in different color spaces and gradients on color channels and gray images. Eventually I settled on two different thresholds - one for yellow lane color and one for white color.

The yellow color is disambuguated by a mask on the hsv color space and is very sensitive to yellow pixels. 

    ```python
    lower_yellow = np.array([10,100,100])
    upper_yellow = np.array([50,200,255])
    ```
Below image shows the pixels picked up using yellow mask on test2.image:

<img src="./output_images/Yellow_test2.jpg" width="480" alt="Yellow Masked Warped test2 Image" />


The white color mask in the RGB space is broad to pick up both white and saturated yellow colors. 

    ```python
    #define range of white in RGB
    lower_white = np.array([200, 200, 0])
    upper_white = np.array([255, 255, 255])
    ```
Below image show the pixels picked up using the white mask on test2.image

<img src="./output_images/Whitetest2.jpg" width="480" alt="White Masked Warped test2 Image" />

The function `threshold_final` also returns a combined binary image with both masks applied. Below for visualization, it is shown with the dst rectangle drawn to illustrate the curvature with respect to straight lines:

<img src="./output_images/BinaryThreshold_DstRect_test2.jpg" width="480" alt="Binary Thresholded Warped test2 Image" />


    
#### 4. Find left, right lane lines, radius of curvature, offset and visualization

All these computations are done in the search_lanes function:

    `color_warp,visual_img = search_lanes(binary_warped[:,:,1])`
    
This function returns a color_warp image to overlay on the original to show the lane. It also returns a visual image that shows how the search was done.
    
The binary thresholded image is then searched for the pixels that belong to the left and right lines of the lane. 

##### i. Cold start - sliding window search with start at base using histogram of bottom half

The first method is a cold start method which uses an initial histogram of the lower half of the images to find starting base points. Then it uses sliding window to go up the image. The window center is adjusted as it finds pixels along the curvature of the line.

<img src="./output_images/LSWindowSlide_test2.jpg" width="480" alt="Window Slide search test2 Image" />


##### ii. Efficient search - search around last polynomial best fit 
Once we have the left and right line pixels disambiguated, we do quadratic polynomial fit. For subsequent searches, for faster operation we search along the last best fit.

<img src="./output_images/LSAroundPoly_test2.jpg" width="480" alt="Search Around Poly test2 Image" />


##### iii.  Best fit - averaging of last 3 fits
We also maintain historical fits/data for smoothing (averaging) to get a better fit. For the challenging videos, averaging over more than 3 is not good.

##### iv.  Sanity check the fits
We check for the sanity of the left and right line widths and expected start positions in the image. If these checks fail we revert back to last fit. If no last fit exists, we revert to default lines.

If sanity check fails 3 times consecutively, we revert to the sliding window search

##### iv.  radii of curvature and offset of vehicle from lane center

Radius of curvature is calculated by fitting the a polynomial in world space after converting the pixels into meters. The coefficients of the fits and the starting y point of the line on the image are then put in the formulae to compute the radii of both left and right lines.

Vehicle Offset is just the differnce between the image center at the last y value and the midpoint of the left and right line starting points. This is also converted into m from pixel units.

    
#### 5. Warp the color_warp back to original image space using inverse perspective matrix (Minv)
    newwarp = cv2.warpPerspective(color_warp, Minv, imshape[1::-1]) 

#### 6. Overlay visualization cotent to be displayed on udistorted image

`result = cv2.addWeighted(undistorted, 1, newwarp, 0.3, 0)`

The final result looks like the image below for test2.jpg. The lane is clearly marked.

<img src="./output_images/Processed_test2.jpg" width="480" alt="Final test2 Image" />



### Pipeline (video)

Here's a [link to my video result](./output_videos/project_video.mp4)
This works flawlessly

I also tried to optimize the thresholding and searching for the challenge vidoes. While it works reasonably well for the challenge video, it falls short on the harder_challenge video

Here's a [link to my challenge video result](./output_videos/challenge_video.mp4)

Here's a [link to my harder challenge video result](./output_videos/harder_challenge_video.mp4)


### Discussion


Two most important aspects of this that can be refined further are the thresholding and searching for the right and left lines. 

For thresholding, I tried various gradients and thresholds on color channels in different color spaces. The shadows, sun shades, bright lights and the texture of the roads introduce edges that can mislead the detections While most approaches worked well on the project video, the challenge video required a lot of optimization.  For the project video even a sobel operator on gray images performed decently. As is evident from the challenge video, when the road has texture whose boundary is parallel and close to the lanes, it is difficult to keep it out of the lane line pixels.

Sanity checks were necessary to recover on the challenge videos. I also reverted to very simple thresholding based on yellow and white color masks.

In the harder challenge videos, besides the sun playing tricks, it was also challenging due to the sharp curvature. We need to incoporate the curvature in the the thresholding and searching. Perhaps this is better solved by using neural networks to classify lane lines than by just conventional image processing.
