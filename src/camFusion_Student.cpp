
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

        // check wether point has been enclosed by one or by multiple boxes
        if (enclosingBoxes.size() == 1)
        {
            // add Lidar point to bounding box
            enclosingBoxes[0]->lidarPoints.push_back(*it1);
        }

    } // eof loop over all Lidar points
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

// associate a given bounding box with the keypoints it contains
void clusterKptMatchesWithROI(BoundingBox &boundingBox, std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> &kptMatches)
{
    // ...
}

// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr,
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg)
{
    // ...
}

void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC)
{
    // ...
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

    // cv::waitKey(0);
    int cFpointsInMultiBBoxes = 0;
    int cFpointsInNoBox = 0;
    int cFpointsInOneBox = 0;
    int pFpointsInMultiBBoxes = 0;
    int pFpointsInNoBox = 0;
    int pFpointsInOneBox = 0;

    for (auto match : matches)
    {

        // cout << "Dmatch: id: " << match.imgIdx << " train: " << match.trainIdx << " query: " << match.queryIdx << endl;
        // sample Dmatch: id: 1656 train: 1301 query: 1656
        // TODO confirm if query or train for previous frame

        // prevFrame.keypoints[match.trainIdx]
        // cout << "matched keypoint coords: " << prevFrame.keypoints[match.trainIdx].pt.x << "," << prevFrame.keypoints[match.trainIdx].pt.y
        //      << "\t" << currFrame.keypoints[match.queryIdx].pt.x << "," << currFrame.keypoints[match.queryIdx].pt.y << endl;
        // cout << "matched keypoint coords: " << prevFrame.keypoints[match.queryIdx].pt.x << "," << prevFrame.keypoints[match.queryIdx].pt.y
        //      << "\t" << currFrame.keypoints[match.trainIdx].pt.x << "," << currFrame.keypoints[match.trainIdx].pt.y << endl;

        // from the check above it seems that the trainIdx goes with current frame. queryIdx with previous....

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
                // cout << "Prev Frame BBox idx :" << pFBBox.boxID << endl;

                // bit stuck here.... need wine...

                if (pFBBox.roi.contains(prevFrame.keypoints[match.queryIdx].pt))
                {
                    // cout << "Prev Frame found in box: " << pFBBox.boxID << endl;
                    pFCandidateBoxes++;

                    cout << "Allocating keypoints in previous frame" << endl;
                    pFBBox.keypoints.push_back(prevFrame.keypoints[match.trainIdx]);
                }
                // cFCandidateBoxList.push_back()
            }

            // cout << "PF candidate boxes: " << to_string(pFCandidateBoxes) << endl;

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

        // vector<int> cFCandidateBoxes;
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
    // for (auto &match : matches)
    // {
    // for (auto &cFBBox : currFrame.boundingBoxes)
    // {
    //     for (auto &pFBBox : prevFrame.boundingBoxes)
    //     {
    //         for (auto kpt : cFBBox.keypoints)
    //         {
    //             if (pFBBox.roi.contains(kpt))
    //             {
    //                             // if (!trainingIds.find(match.trainIdx))
    //         //     trainingIds.push_back(match.trainIdx);
    //         // else
    //         // {
    //         //     cout << "Training ID found more than once: " << match.trainIdx << endl;
    //         // }
    //             }
    //         }
    //     }
    // }

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
