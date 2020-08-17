
#include <iostream>
#include <algorithm>
#include <numeric>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "camFusion.hpp"
#include "dataStructures.h"
#include <unordered_set>

#include "logger.h"

using namespace std;

bool bDebug = false;
bool bfilterViz = false;
Logger logger1("log.txt");

// temporary counter for filenaming for report
int reportCnt = 1; // start at 1 as we only start processing on the 2nd frame (image index =1)
bool bReport = false;

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
void plotPoints(cv::Mat &topviewImg, std::vector<LidarPoint> &lidarPoints, cv::Size worldSize, cv::Size imageSize, int offset_x, cv::Scalar currColor)
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
        int x = (-yw * imageSize.width / worldSize.width) + imageSize.width / 2 + offset_x;

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
}

// quick and dirty plot of Lidar points including filtered ones, for purpose of report
// not going to refactor to avoid duplication as only for reporting
void showLidarPoints(std::vector<LidarPoint> &lidarPoints, std::vector<LidarPoint> &lidarPointsFiltered, bool bWait, string windowName)
{
    cout << "showLidarPoints" << endl;
    // create topview image
    cv::Size imageSize = cv::Size(2000, 2000);
    cv::Size worldSize = cv::Size(3, 12);
    cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(255, 255, 255));

    // create randomized color for current 3D object
    // cv::RNG rng(it1->boxID);
    cv::Scalar currColor = cv::Scalar(255, 0, 0);
    cv::Scalar filteredColor = cv::Scalar(0, 0, 255);

    plotPoints(topviewImg, lidarPoints, worldSize, imageSize, 0, currColor);
    plotPoints(topviewImg, lidarPointsFiltered, worldSize, imageSize, 0, filteredColor);

    // // plot Lidar points into top view image
    int top = 1e8, left = 1e8, bottom = 0.0, right = 0.0;
    float xwmin = 1e8, ywmin = 1e8, ywmax = -1e8;

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

    // resized image as wasnt visible on my setup
    cv::namedWindow(windowName, cv::WINDOW_NORMAL);
    cv::imshow(windowName, topviewImg);
    cv::resizeWindow(windowName, 600, 600);

    // bWait = false;
    if (bWait)
    {
        cv::waitKey(0); // wait for key to be pressed
    }
    bWait = false;
}

// overloaded version of showLidarPoints
// also plots lines showing the x distance that is being used on this frame and previous frame.
void showLidarPoints(std::vector<LidarPoint> &lidarPoints, std::vector<LidarPoint> &lidarPointsFiltered, double lineCurr, double linePrev, double lineOrig, bool bWait, string windowName)
{
    // create topview image
    cv::Size imageSize = cv::Size(2000, 2000);
    cv::Size worldSize = cv::Size(3, 9);
    cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(255, 255, 255));

    // create randomized color for current 3D object
    // cv::RNG rng(it1->boxID);
    cv::Scalar currColor = cv::Scalar(255, 0, 0);
    cv::Scalar filteredColor = cv::Scalar(0, 0, 255);

    plotPoints(topviewImg, lidarPoints, worldSize, imageSize, 0, currColor);
    plotPoints(topviewImg, lidarPointsFiltered, worldSize, imageSize, 0, filteredColor);

    // // plot Lidar points into top view image
    int top = 1e8, left = 1e8, bottom = 0.0, right = 0.0;
    float xwmin = 1e8, ywmin = 1e8, ywmax = -1e8;

    // augment object with some key data
    char str1[200], str2[200];
    sprintf(str1, "currx=%f, prevx=%f, deltax=%f", lineCurr, linePrev, linePrev - lineCurr);
    // sprintf(str1, "test!");
    putText(topviewImg, str1, cv::Point2f(40, 50), cv::FONT_ITALIC, 1.5, cv::Scalar(0, 0, 0));
    // sprintf(str2, "xmin=%2.2f m, yw=%2.2f m", xwmin, ywmax - ywmin);
    // putText(topviewImg, str2, cv::Point2f(left - 250, bottom + 125), cv::FONT_ITALIC, 2, currColor);

    // plot distance markers
    float lineSpacing = 2.0; // gap between distance markers
    int nMarkers = floor(worldSize.height / lineSpacing);
    for (size_t i = 0; i < nMarkers; ++i)
    {
        int y = (-(i * lineSpacing) * imageSize.height / worldSize.height) + imageSize.height;
        cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y), cv::Scalar(255, 0, 0));
    };

    int ycurr = (-lineCurr * imageSize.height / worldSize.height) + imageSize.height;
    int yprev = (-linePrev * imageSize.height / worldSize.height) + imageSize.height;
    // int yorig = (-lineOrig * imageSize.height / worldSize.height) + imageSize.height;

    cv::line(topviewImg, cv::Point(0, ycurr), cv::Point(imageSize.width, ycurr), cv::Scalar(0, 255, 0));
    cv::line(topviewImg, cv::Point(0, yprev), cv::Point(imageSize.width, yprev), cv::Scalar(0, 0, 255));
    // dont plot the 3rd line of now
    // cv::line(topviewImg, cv::Point(0, yorig), cv::Point(imageSize.width, yorig), cv::Scalar(245, 66, 230));

    // resized image as wasnt visible on my setup
    cv::namedWindow(windowName, cv::WINDOW_NORMAL);
    cv::imshow(windowName, topviewImg);
    cv::resizeWindow(windowName, 600, 600);

    // bReport = false;
    if (bReport)
    {

        imwrite("lidar_points" + std::to_string(reportCnt) + ".jpg", topviewImg);
        reportCnt++;
    }

    bWait = false;
    if (bWait)
    {
        cv::waitKey(0); // wait for key to be pressed
    }
    bWait = false;
}

void calculateSD(vector<float> data, float &stddev, float &var, float &mean)
{
    float sum = 0.0;
    float sumOfDiff = 0;

    stddev = 0;
    var = 0;
    for (auto element : data)
    {
        // cout << "ele: " << element << endl;
        sum += element;
    }
    mean = sum / data.size();

    // cout << "mean: " << mean << "sum: " << sum<< endl;
    for (auto element : data)
    {
        sumOfDiff += pow(element - mean, 2);
    }
    var = sumOfDiff / data.size();
    stddev = sqrt(var);

    if (bDebug)
    {
    cout << "calculateSD Debug: data.size(): " << data.size() << " stddev:" << stddev << " var: "
         << " " << var << " mean: " << mean << endl;
    }
}

// associate a given bounding box with the keypoints it contains
void clusterKptMatchesWithROI(BoundingBox &boundingBox, std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> &kptMatches, std::vector<cv::DMatch> &filteredMatches)
{
    // initial analysis of the how the matched points relate to each other
    cout << "clusterKptMatchesWithROI" << endl;
    float var = 0;
    float stddev = 0;
    float mean = 0;

    bDebug = false;
    if (bDebug)
        cv::waitKey(0);

    vector<float> vecEucDists;

    bDebug = false;
    if (bDebug)
        cv::waitKey(0);

    // add all the points, then erase the ones that are outside the threshold
    // allocate the points to the boxes, without filtering
    // calculate the cumlative sum of the euclidean distances to enable calculation of the mean
    for (auto match : kptMatches)
    {
        if (boundingBox.roi.contains(kptsCurr[match.trainIdx].pt))
        {
            boundingBox.keypoints.push_back(kptsCurr[match.trainIdx]);
            boundingBox.kptMatches.push_back(match);

            cv::Point2f diff = kptsCurr[match.trainIdx].pt - kptsPrev[match.queryIdx].pt;
            float euclidean_dist = cv::sqrt(diff.x * diff.x + diff.y * diff.y);
            // cout << "euc " << euclidean_dist << " " << diff.x << " " << diff.y<< endl;
            vecEucDists.push_back(euclidean_dist);
        }
    }

    calculateSD(vecEucDists, stddev, var, mean);
    
    double threshold = 2.5 * stddev;

    // TODO SOMETHING GOING WRONG HERE?
    for (int idx = 0; idx < boundingBox.keypoints.size();)
    {
        // if (abs(vecEucDists[idx] - mean) > threshold)
        if ((vecEucDists[idx] - mean) > threshold)
        {
            // if the entry was erases we dont need to increment the index
            cout << "erasing outlier: " << idx << endl;
            filteredMatches.push_back(boundingBox.kptMatches[idx]);
            vecEucDists.erase(vecEucDists.begin() + idx);
            boundingBox.keypoints.erase(boundingBox.keypoints.begin() + idx);
            boundingBox.kptMatches.erase(boundingBox.kptMatches.begin() + idx);
        }
        else
        {
            idx++;
        }
    }
}

// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr,
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg)
{
    // compute distance ratios between all matched keypoints

    cout << "computeTTCCamera" << endl;
    vector<double> distRatios; // stores the distance ratios for all keypoints between curr. and prev. frame
    for (auto it1 = kptMatches.begin(); it1 != kptMatches.end() - 1; ++it1)
    { // outer kpt. loop
        // get current keypoint and its matched partner in the prev. frame
        cv::KeyPoint kpOuterCurr = kptsCurr.at(it1->trainIdx);
        cv::KeyPoint kpOuterPrev = kptsPrev.at(it1->queryIdx);

        for (auto it2 = kptMatches.begin() + 1; it2 != kptMatches.end(); ++it2)
        {
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

    // use the median distance ratio instead of mean
    std::sort(distRatios.begin(), distRatios.end());
    long medIndex = floor(distRatios.size() / 2.0);
    double medDistRatio = distRatios.size() % 2 == 0 ? (distRatios[medIndex - 1] + distRatios[medIndex]) / 2.0 : distRatios[medIndex];

    double dT = 1 / frameRate;
    TTC = -dT / (1 - medDistRatio);

    bDebug = false;
    if (bDebug)
    {
        cout << "medDistRatio: " << medDistRatio << ", TTC: " << TTC << endl;
    }
}

// Filters Lidar points using a passed standard deviation
// also passes the minimum x value as a reference
void filterPoints(std::vector<LidarPoint> &lidarPoints, std::vector<LidarPoint> &lidarPointsFiltered, float mean_x, float sd, double &minX)
{
    double minXPrev = 1e9;
    double threshold = 2.5 * sd;

    for (auto it = lidarPoints.begin(); it != lidarPoints.end(); ++it)
    {
        if (abs(it->x - mean_x) <= threshold)
        {
            lidarPointsFiltered.push_back(*it);
            minX = minX > it->x ? it->x : minX;
        }
        else
        {
            bDebug = false;
            if (bDebug)
                cout << "outlier: " << it->x << endl;
        }
    }
    bDebug = false;
    if (bDebug)
        cout << "filtered points size: " << lidarPointsFiltered.size() << ", minX: " << minX << endl;
}

void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC)
{
    cout << "computeTTCLidar" << endl;
    float var = 0;
    float sd_curr = 0;
    float sd_prev = 0;
    float mean_x_prev = 0;
    float mean_x_curr = 0;
    float sum_x = 0;
    bDebug = false;
    //
    vector<float> errorsCurr;
    // make vector of the x distances for current frame to use to calculate the mean and std dev
    for (auto it = lidarPointsCurr.begin(); it != lidarPointsCurr.end(); ++it)
    {
        errorsCurr.push_back(it->x);
    }
    calculateSD(errorsCurr, sd_curr, var, mean_x_curr);

    // do the same for the previous frame lidar points
    vector<float> errorsPrev;
    for (auto it = lidarPointsPrev.begin(); it != lidarPointsPrev.end(); ++it)
    {
        errorsPrev.push_back(it->x);
    }
    calculateSD(errorsPrev, sd_prev, var, mean_x_prev);

    // auxiliary variables
    double dT = 0.1;        // time between two measurements in seconds
    double laneWidth = 4.0; // assumed width of the ego lane

    // find closest distance to Lidar points within ego lane
    double minXPrev = 1e9, minXCurr = 1e9;

    // create new vectors to store the filtered points.
    // at this stage code is optimised for visualising so I am using additional vectors rather than reusing the original ones
    std::vector<LidarPoint> lidarPointsCurrFiltered;
    std::vector<LidarPoint> lidarPointsPrevFiltered;

    // run filtering to remover outliers
    filterPoints(lidarPointsPrev, lidarPointsPrevFiltered, mean_x_prev, sd_prev, minXPrev);
    filterPoints(lidarPointsCurr, lidarPointsCurrFiltered, mean_x_curr, sd_curr, minXCurr);
    
    // showLidarPoints(lidarPointsCurr, lidarPointsCurrFiltered, minXCurr, minXPrev, true, windowName);

    // recalculate the mean on the remaining points aftering filtering
    errorsCurr.clear();
    for (auto it = lidarPointsCurrFiltered.begin(); it != lidarPointsCurrFiltered.end(); ++it)
    {
        errorsCurr.push_back(it->x);
        sum_x += it->x;
    }
    double mean_x_curr_filter = sum_x / errorsCurr.size();

    sum_x = 0;
    errorsPrev.clear();
    for (auto it = lidarPointsPrevFiltered.begin(); it != lidarPointsPrevFiltered.end(); ++it)
    {
        errorsPrev.push_back(it->x);
        sum_x += it->x;
    }
    double mean_x_prev_filter = sum_x / errorsPrev.size();

    if (bDebug)
    {
        // display the resultant lidar points
        string windowName = "LIDAR POINTS - curr";
        showLidarPoints(lidarPointsCurr, lidarPointsCurrFiltered, mean_x_curr_filter, mean_x_prev_filter, mean_x_curr, true, windowName);
    }

    // compute TTC from both measurements
    // TTC = minXCurr * dT / (minXPrev - minXCurr);
    TTC = mean_x_curr_filter * dT / (mean_x_prev_filter - mean_x_curr_filter);

    bDebug = false;
    if (bDebug)
    {
        cout << "TTCLidarDebugging" << endl;
        double TTCmean = mean_x_curr * dT / (mean_x_prev - mean_x_curr);
        double TTCmeanfilt = mean_x_curr_filter * dT / (mean_x_prev_filter - mean_x_curr_filter);
        cout << "TTC: " << TTC << "\tminXCurr: " << minXCurr << "\tminXPrev: " << minXPrev << endl;
        cout << "TTCmean: " << TTCmean << "\tmean_x_curr: " << mean_x_curr << "\tmean_x_prev: " << mean_x_prev << endl;
        cout << "TTCmeanfiltered: " << TTCmeanfilt << "\tmean_x_curr: " << mean_x_curr_filter << "\tmean_x_prev: " << mean_x_prev_filter << endl;
    }
}

void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame)
{
    // trainIdx : current frame
    // query: previous
    cout << "matchBoundingBoxes" << endl;
    // some initial debugging
    if (bDebug)
    {
        cout << "Matches size: " << matches.size() << endl;
        cout << "prevFrame Kpts size: " << prevFrame.keypoints.size() << endl;
        cout << "currFrame Kpts size: " << currFrame.keypoints.size() << endl;
        cout << "prevFrame no of Bbox: " << prevFrame.boundingBoxes.size() << endl;
        cout << "currFrame no of Bbox: " << currFrame.boundingBoxes.size() << endl;
    }

    int match_idx = 0;
    int mostMatches = 0;

    // create a 2d vector that can be used to count the matches of each combination of bounding boxes
    // each row will represent a box in current frame, each col will represent a box in prev frame.
    // then the cells will hold the value of the count of matches for that curr frame box and pre frame box
    vector<int> pfBoxMatches(prevFrame.boundingBoxes.size(), 0);
    // 2d vector  vector (rowSize, vector(columnsize, init value))
    vector<vector<int>> vecOfMatchCnts(currFrame.boundingBoxes.size(), pfBoxMatches);

    cout << "vecOfMatchCnts size: " << vecOfMatchCnts.size() << "(curr), " << vecOfMatchCnts[0].size() << "(prev)" << endl;
    bDebug = false;
    if (bDebug)
        cv::waitKey(0);

    // iterate through the matches
    for (auto &match : matches)
    {
        for (auto &cFBBox : currFrame.boundingBoxes)
        {
            // iterate through the current boxes. for each current box, populate an element in a row.
            if (cFBBox.roi.contains(currFrame.keypoints[match.trainIdx].pt))
            {
                // iterate through previous boxes
                for (auto &pFBBox : prevFrame.boundingBoxes)
                {
                    // iterate through the previous boxes. for each previous box, populate an element in a column
                    if (pFBBox.roi.contains(prevFrame.keypoints[match.queryIdx].pt))
                    {
                        // increment the count of matching points for these 2 boxes.
                        vecOfMatchCnts[cFBBox.boxID][pFBBox.boxID]++;
                    }
                }
            }
            match_idx++;
        }
    } // end of matches

    // debug
    bDebug = false;
    if (bDebug)
    {
        cout << "table of matches (each row represents a box in current fream, each column a box in prev frame. row 0, col 3 is curr box 0, prev box 3 matches)" << endl;
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
    // loop hrough the current boxes
    for (int cfBoxIdx = 0; cfBoxIdx < currFrame.boundingBoxes.size(); cfBoxIdx++)
    {
        // set up variables to track best matches
        int pfBoxMatchCnt = -1;
        int pfBoxBestMatchIdx = -1;
        // for each current box check which pf box has most matches
        for (int pfBoxIdx = 0; pfBoxIdx < prevFrame.boundingBoxes.size(); pfBoxIdx++)
        {
            // cout << "\t(" << vecOfMatchCnts[cfBoxIdx][pfBoxIdx] << ")\t";
            if (vecOfMatchCnts[cfBoxIdx][pfBoxIdx] > pfBoxMatchCnt)
            {
                pfBoxMatchCnt = vecOfMatchCnts[cfBoxIdx][pfBoxIdx];
                pfBoxBestMatchIdx = pfBoxIdx;
            }
            cout << vecOfMatchCnts[cfBoxIdx][pfBoxIdx] << "\t";
        }
        // cout << "cfBox: " << cfBoxIdx << " matches pfBox: " << pfBoxBestMatchIdx << endl;

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
            // cout << vecOfMatchCnts[cfBoxIdx][pfBoxIdx] << "\t";
        }
        // if the boxes match in both directions add as a pair
        if (cfBoxIdx == pfBoxTocFBoxCheckIdx)
        {
            // cout << "cfBox: " << cfBoxIdx << " best matches pfBox: " << pfBoxTocFBoxCheckIdx << endl;
            bbBestMatches.insert(pair<int, int>(pfBoxBestMatchIdx, cfBoxIdx));
        }
    }

    bDebug = false;
    if (bDebug)
        cv::waitKey(0);
}
