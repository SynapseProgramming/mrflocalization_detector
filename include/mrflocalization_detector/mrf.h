#ifndef __MRF_H__
#define __MRF_H__

#include <iostream>
#include <cmath>
#include <vector>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sensor_msgs/LaserScan.h>
#include <mrflocalization_detector/Pose.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>

enum MeasurementClass
{
    ALIGNED = 0,
    MISALIGNED = 1,
    UNKNOWN = 2
};

class MRF
{
private:
    // tf names
    std::string mapTopicName;
    std::string mapFrameName;
    // map
    cv::Mat distMap_;
    double mapResolution_;
    Pose mapOrigin_;
    int mapWidth_, mapHeight_;
    bool gotMap_ = false;

    // scan
    bool gotScan_ = false;
    std::string scanTopicName;
    std::string scanFrameName;

    // ros subscribers and publishers
    ros::NodeHandle nh_;
    ros::Subscriber scanSub_;
    ros::Subscriber mapSub_;
    std::string residualErrorsName_;
    std::string failureProbName_, alignedScanName_, misalignedScanName_, unknownScanName_;
    ros::Publisher failureProbPub_, alignedScanPub_, misalignedScanPub_, unknownScanPub_;
    bool publishClassifiedScans_;

    std::string failureProbabilityMarkerName_, markerFrame_;
    ros::Publisher failureProbabilityMarkerPub_;
    bool publishFailureProbabilityMarker_;

    // transforms
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;

    // parameters
    double maxResidualError_;
    double NDMean_, NDVar_, NDNormConst_, EDLambda_;
    int minValidResidualErrorsNum_, maxResidualErrorsNum_;
    int maxLPBComputationNum_;
    int samplingNum_;
    double residualErrorReso_;
    double misalignmentRatioThreshold_, unknownRatioThreshold_;
    std::vector<double> transitionProbMat_;

    sensor_msgs::LaserScan residualErrors_;
    std::vector<double> usedResidualErrors_;
    std::vector<int> usedScanIndices_;
    bool canUpdateResidualErrors_, gotResidualErrors_;
    double failureDetectionHz_;

    // results
    std::vector<std::vector<double>> measurementClassProbabilities_;
    double failureProbability_;
    ros::Time failureProbabilityStamp_;

    // callback function definitions
    void scanCB(const sensor_msgs::LaserScan::ConstPtr &msg);
    void mapCB(const nav_msgs::OccupancyGrid::ConstPtr &msg);

    bool onMap(int u, int v);

    void xy2uv(double x, double y, int *u, int *v);

    inline double calculateNormalDistribution(double e);
    inline double calculateExponentialDistribution(double e);
    inline double calculateUniformDistribution(void);
    inline double getSumOfVector(std::vector<double> vector);
    inline std::vector<double> getHadamardProduct(std::vector<double> vector1, std::vector<double> vector2);
    inline std::vector<double> normalizeVector(std::vector<double> vector);
    inline double getEuclideanNormOfDiffVectors(std::vector<double> vector1, std::vector<double> vector2);
    inline std::vector<double> calculateTransitionMessage(std::vector<double> probs);
    std::vector<std::vector<double>> getLikelihoodVectors(std::vector<double> validResidualErrors);
    std::vector<std::vector<double>> estimateMeasurementClassProbabilities(std::vector<std::vector<double>> likelihoodVectors);
    double predictFailureProbabilityBySampling(std::vector<std::vector<double>> measurementClassProbabilities);
    void setAllMeasurementClassProbabilities(std::vector<double> residualErrors, std::vector<std::vector<double>> measurementClassProbabilities);
    std::vector<int> getResidualErrorClasses(void);

public:
    MRF() : nh_("~"), tfListener(tfBuffer), mapTopicName("map"), mapFrameName("map"), scanTopicName("scan"), scanFrameName("laser_front"), residualErrorsName_("/residual_errors"), failureProbName_("/failure_probability"), alignedScanName_("/aligned_scan_mrf"), misalignedScanName_("/misaligned_scan_mrf"), unknownScanName_("/unknown_scan_mrf"), publishClassifiedScans_(true), failureProbabilityMarkerName_("/failure_probability_marker"), publishFailureProbabilityMarker_(true), markerFrame_("base_link"), NDMean_(0.0), NDVar_(0.01), EDLambda_(2.0), maxResidualError_(1.0), residualErrorReso_(0.05), minValidResidualErrorsNum_(10), maxResidualErrorsNum_(200), maxLPBComputationNum_(1000), samplingNum_(1000), misalignmentRatioThreshold_(0.5), unknownRatioThreshold_(0.7), transitionProbMat_({0.8, 0.0, 0.2, 0.0, 0.8, 0.2, 0.333333, 0.333333, 0.333333}), canUpdateResidualErrors_(true), gotResidualErrors_(false), failureDetectionHz_(10.0)
    {

        nh_.param("map_topic_name", mapTopicName, mapTopicName);
        nh_.param("map_frame_name", mapFrameName, mapFrameName);
        nh_.param("scan_topic_name", scanTopicName, scanTopicName);
        nh_.param("scan_frame_name", scanFrameName, scanFrameName);

        // input and output message names
        nh_.param("residual_errors_name", residualErrorsName_, residualErrorsName_);
        nh_.param("failure_probability_name", failureProbName_, failureProbName_);
        nh_.param("publish_classified_scans", publishClassifiedScans_, publishClassifiedScans_);
        nh_.param("aligned_scan_mrf", alignedScanName_, alignedScanName_);
        nh_.param("misaligned_scan_mrf", misalignedScanName_, misalignedScanName_);
        nh_.param("unknown_scan_mrf", unknownScanName_, unknownScanName_);
        nh_.param("failure_probability_marker_name", failureProbabilityMarkerName_, failureProbabilityMarkerName_);
        nh_.param("publish_failure_probability_marker", publishFailureProbabilityMarker_, publishFailureProbabilityMarker_);
        nh_.param("marker_frame", markerFrame_, markerFrame_);

        // parameters
        nh_.param("normal_distribution_mean", NDMean_, NDMean_);
        nh_.param("normal_distribution_var", NDVar_, NDVar_);
        nh_.param("exponential_distribution_lambda", EDLambda_, EDLambda_);
        nh_.param("max_residual_error", maxResidualError_, maxResidualError_);
        nh_.param("residual_error_resolution", residualErrorReso_, residualErrorReso_);
        nh_.param("min_valid_residual_errors_num", minValidResidualErrorsNum_, minValidResidualErrorsNum_);
        nh_.param("max_residual_errors_num", maxResidualErrorsNum_, maxResidualErrorsNum_);
        nh_.param("max_lpb_computation_num", maxLPBComputationNum_, maxLPBComputationNum_);
        nh_.param("sampling_num", samplingNum_, samplingNum_);
        nh_.param("misalignment_ratio_threshold", misalignmentRatioThreshold_, misalignmentRatioThreshold_);
        nh_.param("unknown_ratio_threshold", unknownRatioThreshold_, unknownRatioThreshold_);
        nh_.param("transition_probability_matrix", transitionProbMat_, transitionProbMat_);

        scanSub_ = nh_.subscribe("/scan_front", 1, &MRF::scanCB, this);
        mapSub_ = nh_.subscribe("/map", 1, &MRF::mapCB, this);

        alignedScanPub_ = nh_.advertise<sensor_msgs::LaserScan>(alignedScanName_, 1);
        misalignedScanPub_ = nh_.advertise<sensor_msgs::LaserScan>(misalignedScanName_, 1);
        unknownScanPub_ = nh_.advertise<sensor_msgs::LaserScan>(unknownScanName_, 1);

        ros::Rate loopRate(10);

        // check map
        int mapFailedCnt = 0;
        while (ros::ok())
        {
            ros::spinOnce();
            if (gotMap_)
                break;
            mapFailedCnt++;
            // 30 sec
            if (mapFailedCnt >= 300)
            {
                ROS_ERROR("Cannot get a map message."
                          " Did you publish the map?"
                          " Expected map topic name is %s\n",
                          mapTopicName.c_str());
                exit(1);
            }
            loopRate.sleep();
        }

        // check scan
        int scanFailedCnt = 0;
        while (ros::ok())
        {
            ros::spinOnce();
            if (gotScan_)
                break;
            scanFailedCnt++;
            if (scanFailedCnt >= 300)
            {
                ROS_ERROR("Cannot get a scan message."
                          " Did you publish the scan?"
                          " Expected scan topic name is %s\n",
                          scanTopicName.c_str());
                exit(1);
            }
            loopRate.sleep();
        }

        // fixed parameters
        NDNormConst_ = 1.0 / sqrt(2.0 * M_PI * NDVar_);
    }

    // getters and setters
    inline void setMaxResidualError(double maxResidualError) { maxResidualError_ = maxResidualError; }
    inline void setNDMean(double NDMean) { NDMean_ = NDMean; }
    inline void setNDVariance(double NDVar) { NDVar_ = NDVar, NDNormConst_ = 1.0 / sqrt(2.0 * M_PI * NDVar_); }
    inline void setEDLambda(double EDLambda) { EDLambda_ = EDLambda; }
    inline void setResidualErrorReso(double residualErrorReso) { residualErrorReso_ = residualErrorReso; }
    inline void setMinValidResidualErrorNum(int minValidResidualErrorNum) { minValidResidualErrorsNum_ = minValidResidualErrorNum; }
    inline void setMaxLPBComputationNum(int maxLPBComputationNum) { maxLPBComputationNum_ = maxLPBComputationNum; }
    inline void setSamplingNum(int samplingNum) { samplingNum_ = samplingNum; }
    inline void setMisalignmentRatioThreshold(double misalignmentRatioThreshold) { misalignmentRatioThreshold_ = misalignmentRatioThreshold; }
    inline void setTransitionProbMat(std::vector<double> transitionProbMat) { transitionProbMat_ = transitionProbMat; }
    inline void setCanUpdateResidualErrors(bool canUpdateResidualErrors) { canUpdateResidualErrors_ = canUpdateResidualErrors; }

    inline double getFailureProbability(void) { return failureProbability_; }
    inline double getMeasurementClassProbabilities(int errorIndex, int measurementClass) { return measurementClassProbabilities_[errorIndex][measurementClass]; }
    inline std::vector<double> getMeasurementClassProbabilities(int errorIndex) { return measurementClassProbabilities_[errorIndex]; }
    inline double getFailureDetectionHz(void) { return failureDetectionHz_; }

    void predictFailureProbability(std::vector<double> ResidualErrors);
    void publishScans();
};

#endif