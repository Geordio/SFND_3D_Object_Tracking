# SFND 3D Object Tracking

## Camera Section Final Project Report

### FP.1 Match 3D Objects

matchBoundingBoxes method matches bounding boxes in current and previous frames by pair the boxes with the highest number of keypoint correspondences.
In order to do this, the matches, bounding boxes in current frame and bounding boxes in previoous frame. A 2D vector is used to build a table of the count of matches between boxes. E.g. for each match, identify in which boxes the trainID and QueryId are located in, and increment the element in the 2D vector that corresponds to that pair of previous frame and current frame bounding boxes.

Once all matches have been processed, the combinations of boxes that have the highest count of matches are selected. Note that I experienced an issue where a current frame bounding box matched 2 boxes in the previous frame. In order to handle these, the bounding box in the previous frame that has the highest count of matches is used. I.e the matching must work both directions.

``` cpp   
    // iterate through the count of matches for boxes in the current frame
    for (int cfBoxIdx = 0; cfBoxIdx < currFrame.boundingBoxes.size(); cfBoxIdx++)
    {
        // set up variables to track best matches
        int pfBoxMatchCnt = -1;
        int pfBoxBestMatchIdx = -1;
        // for each current box check which pf box has most matches
        for (int pfBoxIdx = 0; pfBoxIdx < prevFrame.boundingBoxes.size(); pfBoxIdx++)
        {
            if (vecOfMatchCnts[cfBoxIdx][pfBoxIdx] > pfBoxMatchCnt)
            {
                pfBoxMatchCnt = vecOfMatchCnts[cfBoxIdx][pfBoxIdx];
                pfBoxBestMatchIdx = pfBoxIdx;
            }
        }
        // Note, there is an issue where 1 pf box matches more than 1 cf box, so only add as a matched box, if the tracking works both ways
        // ie check if the pf box has the greatest count of matches with the same cf box
        int cfBoxMatchCnt = -1;
        int pfBoxTocFBoxCheckIdx = -1;
        // check if this matching pfBox matches some other cFBox better. loop through currentframe boxes
        for (int cfBoxChkIdx = 0; cfBoxChkIdx < currFrame.boundingBoxes.size(); cfBoxChkIdx++)
        {
            if (vecOfMatchCnts[cfBoxChkIdx][pfBoxBestMatchIdx] > cfBoxMatchCnt)
            {
                cfBoxMatchCnt = vecOfMatchCnts[cfBoxChkIdx][pfBoxBestMatchIdx];
                pfBoxTocFBoxCheckIdx = cfBoxChkIdx;
            }
        }
        // if the boxes match in both directions add as a pair
        if (cfBoxIdx == pfBoxTocFBoxCheckIdx)
        {
            bbBestMatches.insert(pair<int, int>(pfBoxBestMatchIdx, cfBoxIdx));
        }
    }        
```

### FP.2 Compute Lidar-based TTC

The lidar based TTC is implemented in the computeTTCLidar method.
Additionally, supporting method used are:

* filterPoints: filters the points passed into the method using standard deviation to identify outliers. Any point with a x value that is 2 * standard deviation from the mean is considered an outlier
* calculateSD: calculates the standard deviation
* showLidarPoints: displays a plot of the Lidar points, the filtered lidar points and the x values for this the current frame and the previous frame.
* plotPoints: called by showLidarPoints to plot the individual lidar points

Different approaches were attempted in order to arrive at a reliable solution.

Initially I used the minimum distance from the point cloud of the lead vehicle to the Ego vehicle. I found that this was unreliable, so I additionally added filtering based on standard deviation, with points that are outside of a factor of standard deviation of the mean being treated as outliers and discarded.
This worked in as much as it removed outliers, but there was still significant variance and errors between frames.
In order to overcome this, I used the mean of the filtered points. This works quite well for the sequence in the project, but may have issues if there is a larger variance in the x distances of the LIDAR points, such as if the lead vehicle is a sedan, rather than a hatchbac that has quite a vertical rear end.

The Gif below shows the filtering performance.
Note that there are 2 boxes round the points. The outer box encloses the raw unfiltered points. The inner box encloses the filtered points. Outlier points are shown in blue.
![filtered Lidar](/supporting/filteredLidar/filteredLidar.gif)
As you can see, outliers are sucessfully removed in general.

(create Gif by calling)
```
convert -delay 100 -loop 0 *.jpg filteredLidar.gif
```



### FP.3 Associate Keypoint Correspondences with Bounding Boxes

I iterated over the matches, and check if the points were contained in the region of interest (roi) of each bounding box.
(as proposed in the course lessions.)

In order to remove outliers, I again used the standard deviation, and removed matches that had correspondong points that had a euclidean distance deltas between the previous and current frame of greater than 2.5 times the standard deviation greater than the mean distance.
I only discard points above this threshold, as a lower distance can indicate a good match, especially when there is not a significant change between the ego and lead vehicle relative distance.

I created an overloaded version of clusterKptMatchesWithROI to allow it to return the keypoints that had been filtered out inorder to allow these to be visualised. The code below is then used to visualise the keypoint matches when required.

''' c++
cv::drawMatches((dataBuffer.end() - 2)->cameraImg, (dataBuffer.end() - 2)->keypoints, (dataBuffer.end() - 1)->cameraImg, (dataBuffer.end() - 1)->keypoints, filteredMatches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1), mask, cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
'''

See image below that shows some of the keypoints that were filtered out and discarded.
![filtered Lidar](/supporting/discarded_kpts.png)

a close up side by side comaprison:
![filtered Lidar](/supporting/discarded_kpts_closeup.png)

As you can see, although these were determined to be a pair of matching keypoints on the number plate, the keypoints are located in different positions. Firstly between characters 2 and 3 on the plate, then between 3 and 4. This match was filtered out using the standard deviation of the keypoint matches.


### FP.4 Compute Camera-based TTC

WIth the filtering implentented in the clusterKptMatchesWithROI, the method computeTTCCamera calcualtes the TTC using the median distance ratio.

There are no obvious estimation errors.

### FP.5 Performance Evaluation 1 - Lidar performance

There are not many frames where the estimate is obviously incorrect.
The graph below shows the Lidar TTC for each frame. There are anomolies at frame idx 3 and 17.
![filtered Lidar](/supporting/ttc_lidar.png)

On the fourth frame, the (index3), the TTC increase to 16.9 seconds, when it was approximately 12.5 seconds on the previous frames.
My calculation uses the mean x value of the point cloud after filtering outliers. For this particular frame and the preceeding frame, there is no obvious issues with the point cloud, but the difference in previous and current mean x values changes from 0.062m to 0.047. As the frame rate is constant, this results in the TTC increasing.
The LIDAR plots of these 2 frames are shown below.
Note that the outliers are sucessfully removed in both. The change in mean appears to be the result of the current fram having a higher density of points further away from our ego vehicle, whereas the proceeding frame has a relatively higher density on the edge of the cloud nearer our vehicle. (if you look at the 2nd image blow, you can see that the points in the centre of the inner bounding box are almost continuous as they are density grouped)
Note that the red horizontal line shows the mean x on the previous frame, the green line on the current frame. The closer these lines are together, the less the relative distance between the ego and lead car has changed.
![filtered Lidar](/supporting/filteredLidar/lidar_points2_cropped.jpg)
frame 2 lidar points
![filtered Lidar](/supporting/filteredLidar/lidar_points3_cropped.jpg)


The second example I have chosen shows a similar issue.
For frame 17, the TTC jumps up to 11s when it has been decreasing on the previous frames
My looking at the top down views below, you can see that the distribution of the points has changed, on frame 16, the distribution had an obvious curve, with points in the centre of the y axis being closer to the ego vehicle than the edges, wherease on 17 the curve is less pronouced, with more points distributed more evenly. This effects the mean of the x, making it further away from the ego car, and hence increasing the ttc.
![filtered Lidar](/supporting/filteredLidar/lidar_points16_cropped.jpg)

![filtered Lidar](/supporting/filteredLidar/lidar_points17_cropped.jpg)

Both of these issues could be caused by the rear surface of the lead vehicle not being flat, and changes in the relative poistion of the 2 vehicles to each other, such as the road not being flat, and changes to the pitch of the ego vehicle caused by acceleration and braking. This means that subtly different points on the lead vehicle are detected, changing the overal point distribution.

### FP.6 Performance Evaluation 2 - Camera All detector / descriptor combinations

The project was implenented so that all valid combinations of detector and descriptor could be executed in one run, by setting the variable  bProcessAll to true.
The code will then iterate through all combinations, writing the TTC data to a log file. In the submission state bProcessAll is set to false.

A spreadsheet including all data is at the link [report](/supporting/report_data_sheet.ods)

Below are a fey key graphs.

The graph below shows the performance of camera estimated TTC, with those combinations that performed poorly removed
![Main](/supporting/main.png)

The graph below shows the best performing camera TTC.
![Best Camera](/supporting/best_perf.png)

Below is the FAST BRIEF combination. I had hoped that this would be one of the top performers due to how it performed of the mid term project.
However, you can see an anomoly at Frame ID 5, where the TTC jumps to 21s.
![Fast Brief](/supporting/fast_brief.png)

This can be explained by viewing the matched keypoints.
The image below shows a significant number of keypoints detected off the vehicle, specially highlighted are those detected on the vehicle in the distance in the adjacent lane to the right. These keypoints will increase result in the TTC increasing.

![Adjacent Keypoints](/supporting/adjacent_veh_kpts.png)

This could be handled better my also reducing the size of the bounding box for camera keypoints. The current implementation only reduces the size of the Lidar points.
