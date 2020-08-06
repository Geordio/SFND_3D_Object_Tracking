
#include <iostream>
#include <algorithm>
#include <numeric>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "camFusion.hpp"
#include "dataStructures.h"
#include <unordered_set>

using namespace std;

bool bDebug = true;
bool bfilterViz = true;

// Create groups of Lidar points whose projection into the camera falls into the same bounding box
void clusterLidarWithROI(std::vector<BoundingBox> &boundingBoxes, std::vector<LidarPoint> &lidarPoints, float shrinkFactor, cv::Mat &P_rect_xx, cv::Mat &R_rect_xx, cv::Mat &RT)
{
    if (bDebug)
        cout << "lidar points size: " << lidarPoints.size() << endl;
    // loop over all Lidar points and associate them to a 2D bounding box
    cv::Mat X(4, 1, cv::DataType<double>::type);
    cv::Mat Y(3, 1, cv::DataType<double>::type);

    for (auto it1 = lidarPoints.begin(); it1 != lidarPoints.end(); ++it1)
    {
        // assemble vector for matrix-vector-multiplication
        X.at<double>(0, 0) = it1->x;
        X.at<double>(1, 0) = it1->y;
        X.at<double>(2, 0) = it1->z;
        X.at<double>(3, 0) = 1;

        // project Lidar point into camera
        Y = P_rect_xx * R_rect_xx * RT * X;
        cv::Point pt;
        pt.x = Y.at<double>(0, 0) / Y.at<double>(0, 2); // pixel coordinates
        pt.y = Y.at<double>(1, 0) / Y.at<double>(0, 2);

        vector<vector<BoundingBox>::iterator> enclosingBoxes; // pointers to all bounding boxes which enclose the current Lidar point
        for (vector<BoundingBox>::iterator it2 = boundingBoxes.begin(); it2 != boundingBoxes.end(); ++it2)
        {
            // shrink current bounding box slightly to avoid having too many outlier points around the edges
            cv::Rect smallerBox;
            smallerBox.x = (*it2).roi.x + shrinkFactor * (*it2).roi.width / 2.0;
            smallerBox.y = (*it2).roi.y + shrinkFactor * (*it2).roi.height / 2.0;
            smallerBox.width = (*it2).roi.width * (1 - shrinkFactor);
            smallerBox.height = (*it2).roi.height * (1 - shrinkFactor);

            // check wether point is within current bounding box
            if (smallerBox.contains(pt))
            {
                enclosingBoxes.push_back(it2);
            }

        } // eof loop over all bounding boxes

        // check whether point has been enclosed by one or by multiple boxes
        if (enclosingBoxes.size() == 1)
        {
            // add Lidar point to bounding box
            enclosingBoxes[0]->lidarPoints.push_back(*it1);
        }

    } // eof loop over all Lidar points
}

// Create groups of Lidar points whose projection into the camera falls into the same bounding box
void clusterLidarWithROI(std::vector<BoundingBox> &boundingBoxes, std::vector<LidarPoint> &lidarPoints, float shrinkFactor, cv::Mat &P_rect_xx, cv::Mat &R_rect_xx, cv::Mat &RT, bool bFilter)
{

    clusterLidarWithROI(boundingBoxes, lidarPoints, shrinkFactor, P_rect_xx, R_rect_xx, RT);

    // if filter was requested then post process and erase potential outliers
    // in this case, not actually filtering, have created a new variable of bounding box, lidarPointsFiltered to hold to
    // allow display for purposes of project write up
    // if (bFilter)
    // {
    //     cout "-----------------------------" << endl;
    //     cout << "FILTERING" << endl;
    //     for (auto bbox : boundingBoxes)
    //     {
    //         cout <<"bbox: " bbox.boxID << endl;

    //         // create vector of x values

    //     }

    // }
}

void show3DObjects(std::vector<BoundingBox> &boundingBoxes, cv::Size worldSize, cv::Size imageSize, bool bWait)
{
    // create topview image
    cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(255, 255, 255));

    for (auto it1 = boundingBoxes.begin(); it1 != boundingBoxes.end(); ++it1)
    {
        // create randomized color for current 3D object
        cv::RNG rng(it1->boxID);
        cv::Scalar currColor = cv::Scalar(rng.uniform(0, 150), rng.uniform(0, 150), rng.uniform(0, 150));

        // plot Lidar points into top view image
        int top = 1e8, left = 1e8, bottom = 0.0, right = 0.0;
        float xwmin = 1e8, ywmin = 1e8, ywmax = -1e8;
        for (auto it2 = it1->lidarPoints.begin(); it2 != it1->lidarPoints.end(); ++it2)
        {
            // world coordinates
            float xw = (*it2).x; // world position in m with x facing forward from sensor
            float yw = (*it2).y; // world position in m with y facing left from sensor
            xwmin = xwmin < xw ? xwmin : xw;
            ywmin = ywmin < yw ? ywmin : yw;
            ywmax = ywmax > yw ? ywmax : yw;

            // top-view coordinates
            int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
            int x = (-yw * imageSize.width / worldSize.width) + imageSize.width / 2;

            // find enclosing rectangle
            top = top < y ? top : y;
            left = left < x ? left : x;
            bottom = bottom > y ? bottom : y;
            right = right > x ? right : x;

            // draw individual point
            cv::circle(topviewImg, cv::Point(x, y), 4, currColor, -1);
        }

        // draw enclosing rectangle
        cv::rectangle(topviewImg, cv::Point(left, top), cv::Point(right, bottom), cv::Scalar(0, 0, 0), 2);

        // augment object with some key data
        char str1[200], str2[200];
        sprintf(str1, "id=%d, #pts=%d", it1->boxID, (int)it1->lidarPoints.size());
        putText(topviewImg, str1, cv::Point2f(left - 250, bottom + 50), cv::FONT_ITALIC, 2, currColor);
        sprintf(str2, "xmin=%2.2f m, yw=%2.2f m", xwmin, ywmax - ywmin);
        putText(topviewImg, str2, cv::Point2f(left - 250, bottom + 125), cv::FONT_ITALIC, 2, currColor);
    }

    // plot distance markers
    float lineSpacing = 2.0; // gap between distance markers
    int nMarkers = floor(worldSize.height / lineSpacing);
    for (size_t i = 0; i < nMarkers; ++i)
    {
        int y = (-(i * lineSpacing) * imageSize.height / worldSize.height) + imageSize.height;
        cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y), cv::Scalar(255, 0, 0));
    }

    // display image
    string windowName = "3D Objects";
    // cv::namedWindow(windowName, 1);
    // cv::imshow(windowName, topviewImg);

    // resized image as wasnt visible on my setup
    cv::namedWindow(windowName, cv::WINDOW_NORMAL);
    cv::imshow(windowName, topviewImg);
    cv::resizeWindow(windowName, 600, 600);
    bWait = false;
    if (bWait)
    {
        cv::waitKey(0); // wait for key to be pressed
    }
    bWait = false;
}


// helper function for report creation
// plots lidar points 
void plotPoints(cv::Mat &topviewImg, std::vector<LidarPoint> &lidarPoints, cv::Size worldSize, cv::Size imageSize, cv::Scalar currColor)
{
    // plot Lidar points into top view image
    int top = 1e8, left = 1e8, bottom = 0.0, right = 0.0;
    float xwmin = 1e8, ywmin = 1e8, ywmax = -1e8;
    for (auto it2 = lidarPoints.begin(); it2 != lidarPoints.end(); ++it2)
    {
        // world coordinates
        float xw = (*it2).x; // world position in m with x facing forward from sensor
        float yw = (*it2).y; // world position in m with y facing left from sensor
        xwmin = xwmin < xw ? xwmin : xw;
        ywmin = ywmin < yw ? ywmin : yw;
        ywmax = ywmax > yw ? ywmax : yw;

        // top-view coordinates
        int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
        int x = (-yw * imageSize.width / worldSize.width) + imageSize.width / 2;

        // find enclosing rectangle
        top = top < y ? top : y;
        left = left < x ? left : x;
        bottom = bottom > y ? bottom : y;
        right = right > x ? right : x;

        // draw individual point
        cv::circle(topviewImg, cv::Point(x, y), 4, currColor, -1);
    }

    // draw enclosing rectangle
    cv::rectangle(topviewImg, cv::Point(left, top), cv::Point(right, bottom), cv::Scalar(0, 0, 0), 2);

    // // augment object with some key data
    // char str1[200], str2[200];
    // // sprintf(str1, "id=%d, #pts=%d", it1->boxID, (int)it1->lidarPoints.size());
    // putText(topviewImg, str1, cv::Point2f(left - 250, bottom + 50), cv::FONT_ITALIC, 2, currColor);
    // sprintf(str2, "xmin=%2.2f m, yw=%2.2f m", xwmin, ywmax - ywmin);
    // putText(topviewImg, str2, cv::Point2f(left - 250, bottom + 125), cv::FONT_ITALIC, 2, currColor);
}

// quick and dirty plot of Lidar points including filtered ones, for purpose of report
// not going to refactor to avoid duplication as only for reporting
void showLidarPoints(std::vector<LidarPoint> &lidarPoints, std::vector<LidarPoint> &lidarPointsFiltered, bool bWait)
{
    // create topview image
    cv::Size imageSize = cv::Size(2000, 2000);
    cv::Size worldSize = cv::Size(3, 12);
    cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(255, 255, 255));

    // create randomized color for current 3D object
    // cv::RNG rng(it1->boxID);
    cv::Scalar currColor = cv::Scalar(255, 0, 0);
    cv::Scalar filteredColor = cv::Scalar(0, 0, 255);

    plotPoints(topviewImg, lidarPoints, worldSize, imageSize, currColor);

    plotPoints(topviewImg, lidarPointsFiltered, worldSize, imageSize, filteredColor);

    // // plot Lidar points into top view image
    int top = 1e8, left = 1e8, bottom = 0.0, right = 0.0;
    float xwmin = 1e8, ywmin = 1e8, ywmax = -1e8;
    // for (auto it2 = lidarPoints.begin(); it2 != lidarPoints.end(); ++it2)
    // {
    //     // world coordinates
    //     float xw = (*it2).x; // world position in m with x facing forward from sensor
    //     float yw = (*it2).y; // world position in m with y facing left from sensor
    //     xwmin = xwmin < xw ? xwmin : xw;
    //     ywmin = ywmin < yw ? ywmin : yw;
    //     ywmax = ywmax > yw ? ywmax : yw;

    //     // top-view coordinates
    //     int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
    //     int x = (-yw * imageSize.width / worldSize.width) + imageSize.width / 2;

    //     // find enclosing rectangle
    //     top = top < y ? top : y;
    //     left = left < x ? left : x;
    //     bottom = bottom > y ? bottom : y;
    //     right = right > x ? right : x;

    //     // draw individual point
    //     cv::circle(topviewImg, cv::Point(x, y), 4, currColor, -1);
    // }

    // // draw enclosing rectangle
    // cv::rectangle(topviewImg, cv::Point(left, top), cv::Point(right, bottom), cv::Scalar(0, 0, 0), 2);

    // augment object with some key data
    char str1[200], str2[200];
    // sprintf(str1, "id=%d, #pts=%d", it1->boxID, (int)it1->lidarPoints.size());
    putText(topviewImg, str1, cv::Point2f(left - 250, bottom + 50), cv::FONT_ITALIC, 2, currColor);
    sprintf(str2, "xmin=%2.2f m, yw=%2.2f m", xwmin, ywmax - ywmin);
    putText(topviewImg, str2, cv::Point2f(left - 250, bottom + 125), cv::FONT_ITALIC, 2, currColor);

    // plot distance markers
    float lineSpacing = 2.0; // gap between distance markers
    int nMarkers = floor(worldSize.height / lineSpacing);
    for (size_t i = 0; i < nMarkers; ++i)
    {
        int y = (-(i * lineSpacing) * imageSize.height / worldSize.height) + imageSize.height;
        cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y), cv::Scalar(255, 0, 0));
    }

    // display image
    string windowName = "LIDAR POINTS";
    // cv::namedWindow(windowName, 1);
    // cv::imshow(windowName, topviewImg);

    // resized image as wasnt visible on my setup
    cv::namedWindow(windowName, cv::WINDOW_NORMAL);
    cv::imshow(windowName, topviewImg);
    cv::resizeWindow(windowName, 600, 600);
    bWait = false;
    if (bWait)
    {
        cv::waitKey(0); // wait for key to be pressed
    }
    bWait = false;
}

void calculateSD(vector<float> data, float &stddev, float &var, float &mean)
{
    float sum = 0.0, standardDeviation = 0.0;
    float sumOfDiff = 0;

    stddev = 0;
    var = 0;
    for (auto element : data)
    {
        sum += element;
    }

    mean = sum / data.size();
    cout << "mean: " << mean << endl;
    for (auto element : data)
    {
        sumOfDiff += pow(element - mean, 2);
    }
    var = sumOfDiff / data.size();
    stddev = sqrt(var);
}

// associate a given bounding box with the keypoints it contains
void clusterKptMatchesWithROI(BoundingBox &boundingBox, std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> &kptMatches)
{
    // ...

    // initial analysis of the how the matched points relate to each other
    vector<float> test{-5, 1, 8, 7, 2};
    float var = 0;
    ;
    float stddev = 0;
    float mean = 0;
    calculateSD(test, stddev, var, mean);
    cout << "SD: " << stddev << " ," << var << endl;
    cv::waitKey(0);

    // float point_euc_dist = 0;
    float sum_euc_dist = 0;
    vector<float> errors;
    for (auto match : kptMatches)
    {
        // filtering
        // calculate the euclidean distance between the matched points..

        cv::Point2f diff = kptsCurr[match.trainIdx].pt - kptsPrev[match.queryIdx].pt;
        float point_euc_dist = cv::sqrt(diff.x * diff.x + diff.y * diff.y);
        // sum_euc_dist += point_euc_dist;
        errors.push_back(point_euc_dist);

        // cout << "manually calc ed: " << point_euc_dist << ", match distance: " << match.distance << endl;
    }
    var = 0;
    ;
    stddev = 0;
    calculateSD(errors, stddev, var, mean);
    cout << "SD: " << stddev << " ," << var << endl;

    // float mean_euc_dist = sum_euc_dist / kptMatches.size();
    // cout << "********************************" << endl;
    cv::waitKey(0);

    // // calc std dev
    // for (match : kptMatches)
    // {
    //     var += pow((it->x - mean_x), 2);
    // }
    // sd = sqrt(var);

    for (auto match : kptMatches)
    {

        // filtering
        // calculate the euclidean distance between the matched points..

        cv::Point2f diff = kptsCurr[match.trainIdx].pt - kptsPrev[match.queryIdx].pt;
        float euclidean_dist = cv::sqrt(diff.x * diff.x + diff.y * diff.y);

        // if (euclidean_dist > 3 * stddev

        if (boundingBox.roi.contains(kptsCurr[match.trainIdx].pt))
        {

            boundingBox.keypoints.push_back(kptsCurr[match.trainIdx]);
            boundingBox.kptMatches.push_back(match);
        }
    }
}

// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr,
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg)
{
    // compute distance ratios between all matched keypoints
    vector<double> distRatios; // stores the distance ratios for all keypoints between curr. and prev. frame
    for (auto it1 = kptMatches.begin(); it1 != kptMatches.end() - 1; ++it1)
    { // outer kpt. loop

        // get current keypoint and its matched partner in the prev. frame
        cv::KeyPoint kpOuterCurr = kptsCurr.at(it1->trainIdx);
        cv::KeyPoint kpOuterPrev = kptsPrev.at(it1->queryIdx);

        for (auto it2 = kptMatches.begin() + 1; it2 != kptMatches.end(); ++it2)
        { // inner kpt.-loop

            double minDist = 100.0; // min. required distance

            // get next keypoint and its matched partner in the prev. frame
            cv::KeyPoint kpInnerCurr = kptsCurr.at(it2->trainIdx);
            cv::KeyPoint kpInnerPrev = kptsPrev.at(it2->queryIdx);

            // compute distances and distance ratios
            double distCurr = cv::norm(kpOuterCurr.pt - kpInnerCurr.pt);
            double distPrev = cv::norm(kpOuterPrev.pt - kpInnerPrev.pt);

            if (distPrev > std::numeric_limits<double>::epsilon() && distCurr >= minDist)
            { // avoid division by zero

                double distRatio = distCurr / distPrev;
                distRatios.push_back(distRatio);
            }
        } // eof inner loop over all matched kpts
    }     // eof outer loop over all matched kpts

    // only continue if list of distance ratios is not empty
    if (distRatios.size() == 0)
    {
        TTC = NAN;
        return;
    }

    // STUDENT TASK (replacement for meanDistRatio)
    std::sort(distRatios.begin(), distRatios.end());
    long medIndex = floor(distRatios.size() / 2.0);
    double medDistRatio = distRatios.size() % 2 == 0 ? (distRatios[medIndex - 1] + distRatios[medIndex]) / 2.0 : distRatios[medIndex]; // compute median dist. ratio to remove outlier influence

    double dT = 1 / frameRate;
    TTC = -dT / (1 - medDistRatio);
    // EOF STUDENT TASK
}

void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC)
{
    // ...
    // based on the workshop from the lesson, which does not contain any filtering (other than removing points outside ego lane)
    // TODO, add outlier filtering based on euclidean distance
    // based on the project requirements, clustering seems like overkill, going to use stddev

    // calculate the stddev of x values.

    float var = 0;
    float sd = 0;
    float sum_x = 0;
    //
    vector<float> errors;
    // make vector of the x distances
    for (auto it = lidarPointsCurr.begin(); it != lidarPointsCurr.end(); ++it)
    {
        errors.push_back(it->x);
    }

    for (auto it = lidarPointsCurr.begin(); it != lidarPointsCurr.end(); ++it)
    {
        sum_x += it->x;
    }

    float mean_x = sum_x / lidarPointsCurr.size();
    for (auto it = lidarPointsCurr.begin(); it != lidarPointsCurr.end(); ++it)
    {
        var += pow((it->x - mean_x), 2);
    }
    sd = sqrt(var / lidarPointsCurr.size());

    cout << "------------------------------" << endl;
    cout << "mean x: " << mean_x << ", stddev: " << sd << endl;
    calculateSD(errors, sd, var, mean_x);

    cout << "refactored mean x: " << mean_x << ", stddev: " << sd << endl;
    cout << "------------------------------" << endl;
    cv::waitKey(0);

    // auxiliary variables
    double dT = 0.1;        // time between two measurements in seconds
    double laneWidth = 4.0; // assumed width of the ego lane

    // find closest distance to Lidar points within ego lane

    // 1st, the previous frame
    // note that the way this is implemeneted means that the previous frame is calclauted twice, once when it was the
    // current fram, and again as the previous frame.
    // TODO, fix if I get the chance
    double minXPrev = 1e9, minXCurr = 1e9;
    for (auto it = lidarPointsPrev.begin(); it != lidarPointsPrev.end(); ++it)
    {
        // only use points that are in the ego lane, skip any others
        // not actually needed in this project as there is pre filtering in the main function
        if (abs(it->y) <= laneWidth / 2.0)
        {
            // if the x value is less that the previously smallest x value then use the smaller value
            minXPrev = minXPrev > it->x ? it->x : minXPrev;
        }
    }

    std::vector<LidarPoint> lidarPointsCurrFiltered;

    if (bfilterViz)
    {
        for (auto it = lidarPointsCurr.begin(); it != lidarPointsCurr.end(); ++it)
        {
            if (abs(it->x - mean_x) < 3 * sd)
            {
                lidarPointsCurrFiltered.push_back(*it);
            }
            else
            {
                cout << "outlier: " << it->x << endl;
            }
        }
            cout << "filtered points size: " << lidarPointsCurrFiltered.size() << endl;
    }


    for (auto it = lidarPointsCurr.begin(); it != lidarPointsCurr.end(); ++it)
    {
        if (abs(it->x - mean_x) < 3 * sd)
        {
            // only use points that are in the ego lane, skip any others
            if (abs(it->y) <= laneWidth / 2.0)
            { // 3D point within ego lane?
                minXCurr = minXCurr > it->x ? it->x : minXCurr;
            }
        }
        else
        {
            cout << "outlier: " << it->x << endl;
        }
    }

    // compute TTC from both measurements
    TTC = minXCurr * dT / (minXPrev - minXCurr);

    if (bDebug)
    {
        cout << "TTC: " << TTC << "\tminXCurr: " << minXCurr << "\tminXPrev: " << minXPrev << endl;
    }

    showLidarPoints(lidarPointsCurr, lidarPointsCurrFiltered, true);
}

void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame)
{
    // ...
    // initial thoughts
    //
    // iterate over the matches
    // for each match, check which bounding box its its contained in.
    // OR??? iterate over each DataFrame.boundingbox, and iterate over the keypoints in each bounding box.
    // Is there any real difference to this? Performance or handling outliers / anomalies?
    // how to handle anomalies?
    // for each bounding box, create a list of indexes of possible matching bboxes. the one with the most matches will be the likely one.

    // trainIdx : current frame
    // query: previous
    cout << "-------------------------------------------" << endl;
    // some initial debugging
    if (bDebug)
    {
        cout << "Matches size: " << matches.size() << endl;
        cout << "prevFrame Kpts size: " << prevFrame.keypoints.size() << endl;
        cout << "currFrame Kpts size: " << currFrame.keypoints.size() << endl;
        cout << "prevFrame no of Bbox: " << prevFrame.boundingBoxes.size() << endl;
        cout << "currFrame no of Bbox: " << currFrame.boundingBoxes.size() << endl;
    }

    vector<int> trainingIds;
    int match_idx = 0;
    bDebug = false;
    if (bDebug)
    {
        for (auto match : matches)
        {
            cout << "Dmatch: idx: " << match_idx << " train: " << match.trainIdx << " query: " << match.queryIdx << endl;
            // cv::waitKey(0);
            // if (!trainingIds.find(match.trainIdx))
            //     trainingIds.push_back(match.trainIdx);
            // else
            // {
            //     cout << "Triaing ID found more than once: " << match.trainIdx << endl;
            // }

            if (std::find(trainingIds.begin(), trainingIds.end(), match.trainIdx) != trainingIds.end())
            {
                cout << "Train ID keypt found in more than 1 box: " << match.trainIdx << endl;
            }
            else
            {
                /* code */
                trainingIds.push_back(match.trainIdx);
            }
        }
    }
    bDebug = true;

    cout << "done initial analysis" << endl;
    // cv::waitKey(0);
    int cFpointsInMultiBBoxes = 0;
    int cFpointsInNoBox = 0;
    int cFpointsInOneBox = 0;
    int pFpointsInMultiBBoxes = 0;
    int pFpointsInNoBox = 0;
    int pFpointsInOneBox = 0;

    for (auto match : matches)
    {

        // note
        // because the bounding boxes are independant of the keypoint detection, keypoints can appear in multiple bounding boxes.
        // we could filter out the keypoints that appear in more than 1 bounding box, but I'm not keen on that just yet...
        // I know that the project is focused on ego vehicle lane, but in adjacent lanes there are significant overlap of bounding boxes that
        // enclose keypoints

        vector<int> cFCandidateBoxList;

        int pFCandidateBoxes = 0;
        // iterate through the previous frame
        if (!prevFrame.keyPtsAllocatedToBoxes)
        {
            for (auto pFBBox : prevFrame.boundingBoxes)
            {
                if (pFBBox.roi.contains(prevFrame.keypoints[match.queryIdx].pt))
                {
                    // cout << "Prev Frame found in box: " << pFBBox.boxID << endl;
                    pFCandidateBoxes++;
                    // cout << "Allocating keypoints in previous frame" << endl;
                    pFBBox.keypoints.push_back(prevFrame.keypoints[match.queryIdx]);
                }
                // cFCandidateBoxList.push_back()
            }

            if (pFCandidateBoxes > 1)
            {
                pFpointsInMultiBBoxes++;
            }
            else if (pFCandidateBoxes == 0)
            {
                pFpointsInNoBox++;
            }
            else
            {
                pFpointsInOneBox++;
            }
        }

        int cFCandidateBoxes = 0;
        // iterate through the current frame
        for (auto &cFBBox : currFrame.boundingBoxes)
        {

            // cout << "Current Frame BBox idx :" << cFBBox.boxID << endl;
            if (cFBBox.roi.contains(currFrame.keypoints[match.trainIdx].pt))
            {
                // cout << "Current Frame found in box: " << cFBBox.boxID << endl;
                cFCandidateBoxes++;
                cFBBox.keypoints.push_back(currFrame.keypoints[match.trainIdx]);
            }
        }

        currFrame.keyPtsAllocatedToBoxes = true;
        // cout << "CF candidate boxes: " << to_string(cFCandidateBoxes) << endl;

        if (cFCandidateBoxes > 1)
        {
            cFpointsInMultiBBoxes++;
        }
        else if (cFCandidateBoxes == 0)
        {
            cFpointsInNoBox++;
        }
        else
        {
            cFpointsInOneBox++;
        }
    }

    // at this point we have processed all the matches and allocated each keypoint in the matches to one or more boxes
    // does that make sense to have done? I still have to iterate over the keypoints (or matches) and then each box to find
    // if the box contains the keypoint.
    // there must be a more efficient way. (we've already iterated over boxes and points previously).
    // could combine allocating to boxes and comapring into one set of loops

    // process each box in turn and find the best match in other frame.

    int mostMatches = 0;
    cout << "--------------REVISED-----------" << endl;
    vector<int> cFCandidateBoxList;

    vector<int> boxWithMostMatches(prevFrame.boundingBoxes.size(), 0);
    match_idx = 0;

    vector<int> pfBoxMatches(prevFrame.boundingBoxes.size(), 0);

    // 2d vector  vector (rowSize, vector(columnsize, init value))
    vector<vector<int>> vecOfMatchCnts(currFrame.boundingBoxes.size(), pfBoxMatches);

    cout << "vecOfMatchCnts size: " << vecOfMatchCnts.size() << ", " << vecOfMatchCnts[0].size() << endl;

    cv::waitKey(0);
    for (auto &match : matches)
    {
        for (auto &cFBBox : currFrame.boundingBoxes)
        {

            // cout << "processing box: " << cFBBox.boxID << endl;
            // cout << "processing match: " << match_idx << endl;
            // vector<int> thisBoxToOtherBoxMatchCnt;

            // iterate through the current boxes. for each current box, populate an element in a row.

            if (cFBBox.roi.contains(currFrame.keypoints[match.trainIdx].pt))
            {

                // vector<int> thisBoxToOtherBoxMatchCnt;
                // cFCandidateBoxList.push_back(currFrame.keypoints[match.trainIdx]);
                // cout << "curr Frame found in box: " << pFBBox.boxID << endl;
                // cFCandidateBoxes++;

                // cout << "Allocating keypoints in current frame" << endl;
                // cFBBox.keypoints.push_back(currFrame.keypoints[match.trainIdx]);

                // iterate through previous boxes
                for (auto &pFBBox : prevFrame.boundingBoxes)
                {

                    // iterate through the previous boxes. for each previous box, populate an element in a column
                    if (pFBBox.roi.contains(prevFrame.keypoints[match.queryIdx].pt))
                    {
                        // increment the count of matching points for these 2 boxes.
                        vecOfMatchCnts[cFBBox.boxID][pFBBox.boxID]++;

                        // cout << "Prev Frame found in box: " << pFBBox.boxID << endl;
                        // pFCandidateBoxes++;

                        // cout << "Allocating keypoints in previous frame" << endl;
                        // pFBBox.keypoints.push_back(prevFrame.keypoints[match.queryIdx]);
                    }
                    // cFCandidateBoxList.push_back()
                }
            }
            match_idx++;
        }
    } // end of matches

    // debug
    bDebug = true;
    if (bDebug)
    {
        // vector<vector<int>> vecOfMatchCnts(currFrame.boundingBoxes.size(), vector<int>(prevFrame.boundingBoxes.size(), 0));        cout << "-------------------------" << endl;
        for (int i = 0; i < currFrame.boundingBoxes.size(); i++)
        {
            for (int j = 0; j < prevFrame.boundingBoxes.size(); j++)
            {
                cout << vecOfMatchCnts[i][j] << "\t";
            }
            cout << endl;
        }
        cv::waitKey(0);
    }
    // iterate through the count of matches for boxes in the current frame
    // find index of the max value
    cout << "cf box\tpf box" << endl;
    for (int i = 0; i < currFrame.boundingBoxes.size(); i++)
    {
        // for (int j = 0; j < prevFrame.boundingBoxes.size(); j++)
        // {
        //     // cout << vecOfMatchCnts[i][j] << "\t";
        // }
        // // cout << endl;
        // // int maxElement = *std::max_element(vecOfMatchCnts[i].begin(), vecOfMatchCnts[i].end());
        // // find the index of the max value
        int maxValIndex = std::max_element(vecOfMatchCnts[i].begin(), vecOfMatchCnts[i].end()) - vecOfMatchCnts[i].begin();
        // bbBestMatches.insert(pair<int, int>(i, maxValIndex));
        bbBestMatches.insert(pair<int, int>(i, maxValIndex));

        // cout << "max index: " << maxElementIndex << endl;

        cout << i << "\t" << maxValIndex << endl;
    }
    cv::waitKey(0);

    for (auto pFBBox : prevFrame.boundingBoxes)
    {
        cout << "pfBox Idx: " << pFBBox.boxID << ", keypoints size() " << pFBBox.keypoints.size() << endl;
    }
    for (auto cFBBox : currFrame.boundingBoxes)
    {
        cout << "cfBox Idx: " << cFBBox.boxID << ", keypoints size() " << cFBBox.keypoints.size() << endl;
    }

    cout << "PF matches checked: " << matches.size() << " ,pf points in no box: " << pFpointsInNoBox << " ,pf points in Multi box: " << pFpointsInMultiBBoxes << " ,pf points in 1 box: " << pFpointsInOneBox << endl;
    cout << "CF matches checked: " << matches.size() << " ,cf points in no box: " << cFpointsInNoBox << " ,cf points in Multi box: " << cFpointsInMultiBBoxes << " ,cf points in 1 box: " << cFpointsInOneBox << endl;
}
