**Finding Lane Lines on the Road** 
==================================

### Overview
This project contains a pipline for taking a color image (view of the road ahead)
and creating a modified image that marks the detected lane edges.

![sample](/test_images_output/solidYellowCurve.jpg)

The pipline is applied to a series of test images and views. The outputs of the pipline are stored in the [test_images_output](/test_images_output/)
and [test_videos_output](/test_videos_output/) folders.

The pipeline code is in the [LaneMark.fsx](/LaneMarking/LaneMarking/LaneMark.fsx) file.

### Reflection
 
The pipeline processing is as follows:

#### 1. Grayscale

A copy of the input image is grayscaled for Canny edge detection.
The grayscale copy is masked with a polygon such that only the front center portion is available for edge detection.

![polygon](/img/polygon.png)

The polygon's dimensions are calcluated from the input image dimension and are expressed
as an array of 4 points. The polygon parameters are dependent on the camera used (field of view, focal length, etc.) and 
its mounting position above the ground and hence should be adjusted accordingly.
The [makeMask function](/LaneMarking/LaneMarking/LaneMark.fsx#L34) uses the OpenCV FillConvexPoly function 
internally to fill the mask.

#### 2. Edge detection
The masked grayscale image is processed with OpenCV InRange function.
to isolate the likely lane markings regions. This is done by the 
[isolateLaneMarkers function](/LaneMarking/LaneMarking/LaneMark.fsx#L53)
function of the pipeline.

The mask created by the InRange function is bitwise_and'ed with the grayscale image
to suppress regions other than lane markers. And then Canny edge detection is applied.
See [markEdges function](/LaneMarking/LaneMarking/LaneMark.fsx#L61)

#### 3. Lines
The OpenCV HoughLinesP function is applied to the 'edges' image to find line segments.
This processing is done in the [markLanes pipeline function](/LaneMarking/LaneMarking/LaneMark.fsx#L131).
Ultimately *markLanes* returns an image with lane lines projected onto the image. The many
important intermediate steps this function performs are described next.

##### Line Segment classification
The found line segments are classifed into two buckets - one for each of the two lanes.
First, the slope of each line segment is calculated [See slope](/LaneMarking/LaneMarking/LaneMark.fsx#L26).

Second, the [segmentLanes function](/LaneMarking/LaneMarking/LaneMark.fsx#L83) is used
to bucket the segments. The bucketing is done via [slope ranges](/LaneMarking/LaneMarking/LaneMark.fsx#L72).
Any line segments not within the given ranges are discarded (e.g. horizontal lines).
Note the slope ranges are camera dependent and would have to be adjusted based on 
the camera parameters and it mounting.

##### Segment Averaging
The line segments for each lane bucket are averaged to come up to a single line segment for projection.
The [pipeline function laneOverlay](/LaneMarking/LaneMarking/LaneMark.fsx#L106) performs
the averaging:

- Averge all the slopes
- Average all the points in the line segments to determine a 'mid' point

##### Projection
The 'average' slope and point are used to derive a line equation. This equation is
used to find the projection segment. The OpenCV Line function is used to 
draw projection lines on an empty clone of the input image. This clone 
is merged with the input image using the OpenCV AddWeighted function.


















The project transform images by projecting a marker onto 
the lanes markings identified 

###1. Describe your pipeline. As part of the description, explain how you modified the draw_lines() function.

My pipeline consisted of 5 steps. First, I converted the images to grayscale, then I .... 

In order to draw a single line on the left and right lanes, I modified the draw_lines() function by ...

If you'd like to include images to show how the pipeline works, here is how to include an image: 

![alt text][image1]


###2. Identify potential shortcomings with your current pipeline


One potential shortcoming would be what would happen when ... 

Another shortcoming could be ...


###3. Suggest possible improvements to your pipeline

A possible improvement would be to ...

Another potential improvement could be to ...