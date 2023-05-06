### ORB Features


ORB features are also composed of two parts: ORB keypoints - FAST keypoints with directional component and ORB descriptor - BRIEF descriptors. Its key point is called “oriented FAST”, which is an improved version of the FAST.



##### FAST keypoint

Its main idea is - if a pixel is very different from the neighboring pixels (too bright or too dark), it is more likely to be a corner point.

The algorithm for a single pixel centered at (x,y) is :

1. Select pixel p in the image, assuming its brightness as Ip
2. Set a threshold T (for example, 20% of Ip).
3. Take the pixel p as the center, and select the 16 pixels on a circle with a radius
of 3.
4. If there are consecutive N points on the selected circle whose brightness is greater than Ip +T or less than Ip −T , then the central pixel p can be considered a feature point. N usually takes 12, which is FAST-12. Other commonly used N values are 9 and 11, called FAST-9 and FAST-11, respectively).
5. Iterate through the above four steps on each pi

Problems:

Because radius is fixed (Bresenham circle with radius of 3 pixels), scaling and direction is an issue, e.g - corner detected in image 1 in image 2 will be zoomed in (if moving towards the corner). FAST is added with a scale pyramid and orientation component.

Solution:

1. Scale invariance with pyramid scheme: For each layer up, the image is scaled with a fixed ratio so that we have images of different resolutions. The smaller image can be seen as a scene viewed from a distance. In the feature matching algorithm, we can match images on different layers to achieve scale invariance. For example, if the camera moves backward, we should find a match in the upper layer of the previous image pyramid and the lower layer of the next image.

2. In terms of rotation, we calculate the gray centroid of the image near the feature point. The so-called centroid refers to the gray value of the image block as the center of weight. Obtain orientation theta from moments m01 and m10.




##### BRIEF descriptor  

BRIEF is a binary descriptor. Its description vector consists of many zeros and ones, which encode the size relationship between two random pixels near the key point (such as p and q): If p is greater than q, then take 1, otherwise take 0. If we take 128 such p, q pairs, we will finally get a 128-dimensional vector [44] consisting of 0s and 1s. The BRIEF implements the comparison of randomly selected points, which is very fast. Since it expresses in binary, it is also very convenient to store and suitable for real-time image matching. The original BRIEF descriptor does not have rotation invariance, so it is easy to get lost when the image is rotated. The ORB calculates the direction of the key points in the FAST feature point extraction stage. The direction information can be used to calculate the Steer BRIEF feature after the rotation so that the ORB descriptor has better rotation invariance.