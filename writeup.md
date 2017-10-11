
# Self-Driving Car Engineer Nanodegree


## Project: **Finding Lane Lines on the Road** 
***
Project writeup, Iker Diez Arancibia  2017/05/29

---

### Inroduction and goals

---

Through the **Finding Lane Lines on the Road** project I have learned to set up a python environment on my computer and have adapted my programming skills to python 3.0 

The goals of this project are the following:
* Make a pipeline that finds lane lines on the road
* Apply the pipeline to images
* Apply the pipeline to videos with straight or slightly curved paths


[//]: # (Image References)

[image1]: /test_images_out/whiteCarLaneSwitch.jpg "Full processed"


---

### Reflection

### 1. Describe your pipeline. As part of the description, explain how you modified the draw_lines() function.

My pipeline consisted of 6 steps. First, I converted the images to grayscale, then I blurred the images using the gaussian_blur() function provided by the course reducing the noise introduced in the canny edge analysis, which is the third step. Once those steps are done, a mask for retaining the area of interest is applied. The last step before modifying the draw_lines() functions is to apply the hough analysis which converts the high gradient showed up by the canny edge analysis into a lines array containing those line points.

Finnaly I modified the provided draw_lines() function in order to average and extrapolate the lines to the entire interest area, taking as input the Hough analysis output array and the image in which the lines are going to be drawed. In order to separate the points belonging to each right and left lines, I have done a linear regresion using the numpy.polyfit feature between each line point pairs, obtaining the slope of the line between them. Evaluating if the slope is positive or negative and if the points are on the left or right side of the image I cleared out if the points have to be saved in the right line lists (r_x and r_y) or the left line lists(l_x and l_y).

Once the points are classified between both lines, another linear regresion is calculated to fit optimally all the point belonging to each side lines. In order  to extrapolate the lines to the entire extension of the interest area, the x coordinates of the lowest and highest y values of the area are calculated using the ecuation of the line and the slope obtained in the linear regresion. Therefore, after all the analysis the fully processed image looks like the one below.

 ![alt text][image1]




### 2. Identify potential shortcomings with your current pipeline


One potential shortcoming would be what happens when the masked area does not adapt to the road ahead as happens when applying the code to the challenge video. In this case, the interest area is filled with noise that my code can not filter yet, this noise displaces the lines as it can be seen in the left line of the challenge.

Another shortcoming could be the fact that the lines tremble more than the ones showed on the example video  which when applied to an actual car would introduce sporadicall alterations in the car's path, which would be very unconfortable for the passengers.

### 3. Suggest possible improvements to your pipeline

A possible improvement would be to reduce the trembling explained before introducing a "weighted" interpolation between the slope calculated in the actual frame of the video and the previous one, smoothing thoose trembles.

Another potential improvement could be to dynamically adapt the interest area depending the driving situation.

