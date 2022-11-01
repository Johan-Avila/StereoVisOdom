#pragma once

#ifndef ULTILITY_H
#define ULTILITY_H

#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Int8.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>

#include <opencv2/opencv.hpp>

#include <iostream>
#include <fstream>

using namespace std;

class ParamServer
{
public:
    ros::NodeHandle nh;
    string pwd_base_topics = "/stereo_vis_odom/topics/";
    string pwd_base_orb = "/stereo_vis_odom/orb/";
    string pwd_base_bf = "/stereo_vis_odom/bf/";
    string pwd_base_fil_StereoGeometry = "/stereo_vis_odom/filters/StereoGeometry/";
    string pwd_base_solvePnPRansac = "/stereo_vis_odom/solvePnPRansac/";
    string pwd_base_calib_matrixProjection = "/stereo_vis_odom/stereoCalibration/matrixProjection/";

    bool pnp_useExtrinsicGuess;
    int pnp_iterationsCount;
    float pnp_reprojectionError;
    double pnp_confidence;
    int pnp_flags;

    ParamServer()
    {
        string pwd_pnp_useExtrinsicGuess;
        pwd_pnp_useExtrinsicGuess.append(pwd_base_solvePnPRansac);
        pwd_pnp_useExtrinsicGuess.append("useExtrinsicGuess");

        string pwd_pnp_iterationsCount;
        pwd_pnp_iterationsCount.append(pwd_base_solvePnPRansac);
        pwd_pnp_iterationsCount.append("iterationsCount");

        string pwd_pnp_reprojectionError;
        pwd_pnp_reprojectionError.append(pwd_base_solvePnPRansac);
        pwd_pnp_reprojectionError.append("reprojectionError");

        string pwd_pnp_confidence;
        pwd_pnp_confidence.append(pwd_base_solvePnPRansac);
        pwd_pnp_confidence.append("confidence");

        string pwd_pnp_flags;
        pwd_pnp_flags.append(pwd_base_solvePnPRansac);
        pwd_pnp_flags.append("flags");

        nh.param<bool>(pwd_pnp_useExtrinsicGuess, pnp_useExtrinsicGuess, false);
        nh.param<int>(pwd_pnp_iterationsCount, pnp_iterationsCount, 100);
        nh.param<float>(pwd_pnp_reprojectionError, pnp_reprojectionError, 2.0);
        nh.param<double>(pwd_pnp_confidence, pnp_confidence, 0.99);
        nh.param<int>(pwd_pnp_flags, pnp_flags, 2);
    }

    string get_topic(const char *name)
    {
        string pwd_topic;
        pwd_topic.append(pwd_base_topics);
        pwd_topic.append(name);

        string topics;
        nh.param<string>(pwd_topic, topics, "");
        return topics;
    }

    cv::Ptr<cv::ORB> create_orb()
    {
        string pwd_nfeatures;
        pwd_nfeatures.append(pwd_base_orb);
        pwd_nfeatures.append("nfeatures");

        string pwd_scaleFactor;
        pwd_scaleFactor.append(pwd_base_orb);
        pwd_scaleFactor.append("scaleFactor");

        string pwd_nlevels;
        pwd_nlevels.append(pwd_base_orb);
        pwd_nlevels.append("nlevels");

        string pwd_edgeThreshold;
        pwd_edgeThreshold.append(pwd_base_orb);
        pwd_edgeThreshold.append("edgeThreshold");

        string pwd_firstLevel;
        pwd_firstLevel.append(pwd_base_orb);
        pwd_firstLevel.append("firstLevel");

        string pwd_WTA_K;
        pwd_WTA_K.append(pwd_base_orb);
        pwd_WTA_K.append("WTA_K");

        string pwd_scoreType;
        pwd_scoreType.append(pwd_base_orb);
        pwd_scoreType.append("scoreType");

        string pwd_patchSize;
        pwd_patchSize.append(pwd_base_orb);
        pwd_patchSize.append("patchSize");

        string pwd_fastThreshold;
        pwd_fastThreshold.append(pwd_base_orb);
        pwd_fastThreshold.append("fastThreshold");

        int nfeatures;
        float scaleFactor;
        int nlevels;
        int edgeThreshold;
        int firstLevel;
        int WTA_K;
        int scoreType;
        int patchSize;
        int fastThreshold;

        nh.param<int>(pwd_nfeatures, nfeatures, 500);
        nh.param<float>(pwd_scaleFactor, scaleFactor, 1.2);
        nh.param<int>(pwd_nlevels, nlevels, 8);
        nh.param<int>(pwd_edgeThreshold, edgeThreshold, 31);
        nh.param<int>(pwd_firstLevel, firstLevel, 0);
        nh.param<int>(pwd_WTA_K, WTA_K, 2);
        nh.param<int>(pwd_scoreType, scoreType, 0);
        nh.param<int>(pwd_patchSize, patchSize, 31);
        nh.param<int>(pwd_fastThreshold, fastThreshold, 20);

        cv::Ptr<cv::ORB> orb;
        orb = cv::ORB::create(nfeatures,
                              scaleFactor,
                              nlevels,
                              edgeThreshold,
                              firstLevel,
                              WTA_K,
                              cv::ORB::HARRIS_SCORE,
                              patchSize,
                              fastThreshold);
        return orb;
    }

    cv::Ptr<cv::BFMatcher> create_bf()
    {
        string pwd_normType;
        pwd_normType.append(pwd_base_bf);
        pwd_normType.append("normType");

        string pwd_crossCheck;
        pwd_crossCheck.append(pwd_base_bf);
        pwd_crossCheck.append("crossCheck");

        int normType;
        bool crossCheck;

        nh.param<int>(pwd_normType, normType, 6);
        nh.param<bool>(pwd_crossCheck, crossCheck, false);

        cv::Ptr<cv::BFMatcher> bf;
        bf = cv::BFMatcher::create(
            normType,
            crossCheck);
        return bf;
    }

    int get_fil_StereoGeometry(const char *name)
    {
        string pwd_value;
        pwd_value.append(pwd_base_fil_StereoGeometry);
        pwd_value.append(name);

        int value;
        nh.param<int>(pwd_value, value, 0);
        return value;
    }

    cv::Mat get_calib_matrixProjection(const char *name)
    {
        string pwd_mp;
        pwd_mp.append(pwd_base_calib_matrixProjection);
        pwd_mp.append(name);

        vector<double> mpV;
        cv::Mat mp(3, 4, cv::DataType<double>::type);

        nh.param<vector<double>>(pwd_mp, mpV, vector<double>());

        mp.at<double>(0, 0) = mpV[0];
        mp.at<double>(0, 1) = mpV[1];
        mp.at<double>(0, 2) = mpV[2];
        mp.at<double>(0, 3) = mpV[3];

        mp.at<double>(1, 0) = mpV[4];
        mp.at<double>(1, 1) = mpV[5];
        mp.at<double>(1, 2) = mpV[6];
        mp.at<double>(1, 3) = mpV[7];

        mp.at<double>(2, 0) = mpV[8];
        mp.at<double>(2, 1) = mpV[9];
        mp.at<double>(2, 2) = mpV[10];
        mp.at<double>(2, 3) = mpV[11];

        return mp;
    }
};
#endif