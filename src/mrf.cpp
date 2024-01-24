#include <mrflocalization_detector/mrf.h>

void MRF::scanCB(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    residualErrors_ = *msg;

    std::vector<double> validResidualErrors;
    for (int i = 0; i < msg->ranges.size(); i++)
    {
        double distance = msg->ranges[i];
        if (distance <= msg->range_min || msg->range_max <= distance)
        {
            validResidualErrors.push_back(-1.0);
            continue;
        }

        double d = (double)i * msg->angle_increment + msg->angle_min;
        double xx = msg->ranges[i] * cos(d);
        double yy = msg->ranges[i] * sin(d);
        // translate laser points to be wrt to map frame
        geometry_msgs::PointStamped point_in;
        geometry_msgs::PointStamped point_out;
        point_in.header.frame_id = scanFrameName;
        point_in.point.x = xx;
        point_in.point.y = yy;
        try
        {
            point_out = tfBuffer.transform(point_in, mapFrameName);
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
            return;
        }
        double mapX = point_out.point.x;
        double mapY = point_out.point.y;
        // image width and image height are pixel indices
        int imageWidth = 0;
        int imageHeight = 0;
        xy2uv(mapX, mapY, &imageWidth, &imageHeight);
        if (onMap(imageWidth, imageHeight))
        {
            double residualError = distMap_.at<float>(imageHeight, imageWidth);
            validResidualErrors.push_back(residualError);
        }
        else
        {
            validResidualErrors.push_back(-1.0);
        }
    }

    // pass as aruments
    predictFailureProbability(validResidualErrors);

    gotScan_ = true;
}

void MRF::mapCB(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{

    if (gotMap_)
        return;
    // perform distance transform to build the distance field
    mapWidth_ = msg->info.width;
    mapHeight_ = msg->info.height;
    mapResolution_ = msg->info.resolution;
    cv::Mat binMap(mapHeight_, mapWidth_, CV_8UC1);
    for (int v = 0; v < mapHeight_; v++)
    {
        for (int u = 0; u < mapWidth_; u++)
        {
            int node = v * mapWidth_ + u;
            int val = msg->data[node];
            if (val == 100)
                binMap.at<uchar>(v, u) = 0;
            else
                binMap.at<uchar>(v, u) = 1;
        }
    }
    cv::Mat distMap(mapHeight_, mapWidth_, CV_32FC1);
    cv::distanceTransform(binMap, distMap, cv::DIST_L2, 5);
    for (int v = 0; v < mapHeight_; v++)
    {
        for (int u = 0; u < mapWidth_; u++)
        {
            float d = distMap.at<float>(v, u) * (float)mapResolution_;
            distMap.at<float>(v, u) = d;
        }
    }
    distMap_ = distMap;
    tf2::Quaternion q(msg->info.origin.orientation.x,
                      msg->info.origin.orientation.y,
                      msg->info.origin.orientation.z,
                      msg->info.origin.orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);
    mapOrigin_.setX(msg->info.origin.position.x);
    mapOrigin_.setY(msg->info.origin.position.y);
    mapOrigin_.setYaw(yaw);
    gotMap_ = true;
}

bool MRF::onMap(int u, int v)
{
    if (0 <= u && u < mapWidth_ && 0 <= v && v < mapHeight_)
        return true;
    else
        return false;
}

void MRF::xy2uv(double x, double y, int *u, int *v)
{
    double dx = x - mapOrigin_.getX();
    double dy = y - mapOrigin_.getY();
    double yaw = -mapOrigin_.getYaw();
    double xx = dx * cos(yaw) - dy * sin(yaw);
    double yy = dx * sin(yaw) + dy * cos(yaw);
    *u = (int)(xx / mapResolution_);
    *v = (int)(yy / mapResolution_);
}

inline double MRF::calculateNormalDistribution(double e)
{
    return (0.95 * (2.0 * NDNormConst_ * exp(-((e - NDMean_) * (e - NDMean_)) / (2.0 * NDVar_))) + 0.05 * (1.0 / maxResidualError_)) * residualErrorReso_;
}

inline double MRF::calculateExponentialDistribution(double e)
{
    return (0.95 * (1.0 / (1.0 - exp(-EDLambda_ * maxResidualError_))) * EDLambda_ * exp(-EDLambda_ * e) + 0.05 * (1.0 / maxResidualError_)) * residualErrorReso_;
}

inline double MRF::calculateUniformDistribution(void)
{
    return (1.0 / maxResidualError_) * residualErrorReso_;
}

inline double MRF::getSumOfVector(std::vector<double> vector)
{
    double sum = 0.0;
    for (int i = 0; i < (int)vector.size(); i++)
        sum += vector[i];
    return sum;
}

inline std::vector<double> MRF::getHadamardProduct(std::vector<double> vector1, std::vector<double> vector2)
{
    for (int i = 0; i < (int)vector1.size(); i++)
        vector1[i] *= vector2[i];
    return vector1;
}

inline std::vector<double> MRF::normalizeVector(std::vector<double> vector)
{
    double sum = getSumOfVector(vector);
    for (int i = 0; i < (int)vector.size(); i++)
        vector[i] /= sum;
    return vector;
}

inline double MRF::getEuclideanNormOfDiffVectors(std::vector<double> vector1, std::vector<double> vector2)
{
    double sum = 0.0;
    for (int i = 0; i < (int)vector1.size(); i++)
    {
        double diff = vector1[i] - vector2[i];
        sum += diff * diff;
    }
    return sqrt(sum);
}

inline std::vector<double> MRF::calculateTransitionMessage(std::vector<double> probs)
{
    std::vector<double> message(3);
    std::vector<double> tm = transitionProbMat_;
    message[ALIGNED] = tm[ALIGNED] * probs[ALIGNED] + tm[MISALIGNED] * probs[MISALIGNED] + tm[UNKNOWN] * probs[UNKNOWN];
    message[MISALIGNED] = tm[ALIGNED + 3] * probs[ALIGNED] + tm[MISALIGNED + 3] * probs[MISALIGNED] + tm[UNKNOWN + 3] * probs[UNKNOWN];
    message[UNKNOWN] = tm[ALIGNED + 6] * probs[ALIGNED] + tm[MISALIGNED + 6] * probs[MISALIGNED] + tm[UNKNOWN + 6] * probs[UNKNOWN];
    return message;
}

std::vector<std::vector<double>> MRF::getLikelihoodVectors(std::vector<double> validResidualErrors)
{
    std::vector<std::vector<double>> likelihoodVectors((int)validResidualErrors.size());
    double pud = calculateUniformDistribution();
    for (int i = 0; i < (int)likelihoodVectors.size(); i++)
    {
        likelihoodVectors[i].resize(3);
        likelihoodVectors[i][ALIGNED] = calculateNormalDistribution(validResidualErrors[i]);
        likelihoodVectors[i][MISALIGNED] = calculateExponentialDistribution(validResidualErrors[i]);
        likelihoodVectors[i][UNKNOWN] = pud;
        likelihoodVectors[i] = normalizeVector(likelihoodVectors[i]);
    }
    return likelihoodVectors;
}

std::vector<std::vector<double>> MRF::estimateMeasurementClassProbabilities(std::vector<std::vector<double>> likelihoodVectors)
{
    std::vector<std::vector<double>> measurementClassProbabilities = likelihoodVectors;
    for (int i = 0; i < (int)measurementClassProbabilities.size(); i++)
    {
        for (int j = 0; j < (int)measurementClassProbabilities.size(); j++)
        {
            if (i == j)
                continue;
            std::vector<double> message = calculateTransitionMessage(likelihoodVectors[j]);
            measurementClassProbabilities[i] = getHadamardProduct(measurementClassProbabilities[i], message);
            measurementClassProbabilities[i] = normalizeVector(measurementClassProbabilities[i]);
        }
        measurementClassProbabilities[i] = normalizeVector(measurementClassProbabilities[i]);
    }

    double variation = 0.0;
    int idx1 = rand() % (int)measurementClassProbabilities.size();
    std::vector<double> message(3);
    message = likelihoodVectors[idx1];
    int checkStep = maxLPBComputationNum_ / 20;
    for (int i = 0; i < maxLPBComputationNum_; i++)
    {
        int idx2 = rand() % (int)measurementClassProbabilities.size();
        int cnt = 0;
        for (;;)
        {
            if (idx2 != idx1)
                break;
            idx2 = rand() % (int)measurementClassProbabilities.size();
            cnt++;
            if (cnt >= 10)
                break;
        }
        message = calculateTransitionMessage(message);
        message = getHadamardProduct(likelihoodVectors[idx2], message);
        std::vector<double> measurementClassProbabilitiesPrev = measurementClassProbabilities[idx2];
        measurementClassProbabilities[idx2] = getHadamardProduct(measurementClassProbabilities[idx2], message);
        measurementClassProbabilities[idx2] = normalizeVector(measurementClassProbabilities[idx2]);
        double diffNorm = getEuclideanNormOfDiffVectors(measurementClassProbabilities[idx2], measurementClassProbabilitiesPrev);
        variation += diffNorm;
        if (i >= checkStep && i % checkStep == 0 && variation < 10e-6)
            break;
        else if (i >= checkStep && i % checkStep == 0)
            variation = 0.0;
        message = measurementClassProbabilities[idx2];
        idx1 = idx2;
    }
    return measurementClassProbabilities;
}

double MRF::predictFailureProbabilityBySampling(std::vector<std::vector<double>> measurementClassProbabilities)
{
    int failureCnt = 0;
    for (int i = 0; i < samplingNum_; i++)
    {
        int misalignedNum = 0, validMeasurementNum = 0;
        int measurementNum = (int)measurementClassProbabilities.size();
        for (int j = 0; j < measurementNum; j++)
        {
            double darts = (double)rand() / ((double)RAND_MAX + 1.0);
            double validProb = measurementClassProbabilities[j][ALIGNED] + measurementClassProbabilities[j][MISALIGNED];
            if (darts > validProb)
                continue;
            validMeasurementNum++;
            if (darts > measurementClassProbabilities[j][ALIGNED])
                misalignedNum++;
        }
        double misalignmentRatio = (double)misalignedNum / (double)validMeasurementNum;
        double unknownRatio = (double)(measurementNum - validMeasurementNum) / (double)measurementNum;
        if (misalignmentRatio >= misalignmentRatioThreshold_ || unknownRatio >= unknownRatioThreshold_)
            failureCnt++;
    }
    double p = (double)failureCnt / (double)samplingNum_;
    return p;
}

void MRF::setAllMeasurementClassProbabilities(std::vector<double> residualErrors, std::vector<std::vector<double>> measurementClassProbabilities)
{
    measurementClassProbabilities_.resize((int)residualErrors.size());
    int idx = 0;
    for (int i = 0; i < (int)measurementClassProbabilities_.size(); i++)
    {
        measurementClassProbabilities_[i].resize(3);
        if (0.0 <= residualErrors[i] && residualErrors[i] <= maxResidualError_)
        {
            measurementClassProbabilities_[i] = measurementClassProbabilities[idx];
            idx++;
        }
        else
        {
            measurementClassProbabilities_[i][ALIGNED] = 0.00005;
            measurementClassProbabilities_[i][MISALIGNED] = 0.00005;
            measurementClassProbabilities_[i][UNKNOWN] = 0.9999;
        }
    }
}

std::vector<int> MRF::getResidualErrorClasses(void)
{
    int size = (int)measurementClassProbabilities_.size();
    std::vector<int> residualErrorClasses(size);
    for (int i = 0; i < size; i++)
    {
        double alignedProb = measurementClassProbabilities_[i][ALIGNED];
        double misalignedProb = measurementClassProbabilities_[i][MISALIGNED];
        double unknownProb = measurementClassProbabilities_[i][UNKNOWN];
        if (alignedProb > misalignedProb && alignedProb > unknownProb)
            residualErrorClasses[i] = ALIGNED;
        else if (misalignedProb > alignedProb && misalignedProb > unknownProb)
            residualErrorClasses[i] = MISALIGNED;
        else
            residualErrorClasses[i] = UNKNOWN;
    }
    return residualErrorClasses;
}

void MRF::predictFailureProbability(std::vector<double> ResidualErrors)
{

    std::vector<double> validResidualErrors;
    std::vector<int> validScanIndices;

    // create a new array only containing valid residual errors(closest distance <=1m)
    for (int i = 0; i < (int)ResidualErrors.size(); ++i)
    {
        double e = ResidualErrors[i];
        if (0.0 <= e && e <= maxResidualError_)
        {
            validResidualErrors.push_back(e);
            validScanIndices.push_back(i);
        }
    }

    int validResidualErrorsSize = (int)validResidualErrors.size();
    if (validResidualErrorsSize <= minValidResidualErrorsNum_)
    {
        std::cerr << "WARNING: Number of validResidualErrors is less than the expected threshold number."
                  << " The threshold is " << minValidResidualErrorsNum_ << ", but the number of validResidualErrors " << validResidualErrorsSize << "." << std::endl;
        failureProbability_ = -1.0;
        return;
    }
    else if (validResidualErrorsSize <= maxResidualErrorsNum_)
    {
        usedResidualErrors_ = validResidualErrors;
        usedScanIndices_ = validScanIndices;
    }
    else
    { // undersample the residual errors

        usedResidualErrors_.resize(maxResidualErrorsNum_);
        usedScanIndices_.resize(maxResidualErrorsNum_);
        for (int i = 0; i < maxResidualErrorsNum_; ++i)
        {
            int idx = rand() % (int)validResidualErrors.size();
            usedResidualErrors_[i] = validResidualErrors[idx];
            usedScanIndices_[i] = validScanIndices[idx];
            validResidualErrors.erase(validResidualErrors.begin() + idx);
            validScanIndices.erase(validScanIndices.begin() + idx);
        }
    }
    std::vector<std::vector<double>> likelihoodVectors = getLikelihoodVectors(usedResidualErrors_);
    std::vector<std::vector<double>> measurementClassProbabilities = estimateMeasurementClassProbabilities(likelihoodVectors);
    std::vector<double> first = measurementClassProbabilities[0];
    setAllMeasurementClassProbabilities(usedResidualErrors_, measurementClassProbabilities);
    failureProbability_ = predictFailureProbabilityBySampling(measurementClassProbabilities_);
    std::cout << "Failure Probability: " << failureProbability_ << "\n";
    publishScans();
}

void MRF::publishScans()
{
    std::vector<int> residualErrorClasses = getResidualErrorClasses();
    sensor_msgs::LaserScan alignedScan, misalignedScan, unknownScan;
    alignedScan.header = misalignedScan.header = unknownScan.header = residualErrors_.header;
    alignedScan.range_min = misalignedScan.range_min = unknownScan.range_min = residualErrors_.range_min;
    alignedScan.range_max = misalignedScan.range_max = unknownScan.range_max = residualErrors_.range_max;
    alignedScan.angle_min = misalignedScan.angle_min = unknownScan.angle_min = residualErrors_.angle_min;
    alignedScan.angle_max = misalignedScan.angle_max = unknownScan.angle_max = residualErrors_.angle_max;
    alignedScan.angle_increment = misalignedScan.angle_increment = unknownScan.angle_increment = residualErrors_.angle_increment;
    alignedScan.time_increment = misalignedScan.time_increment = unknownScan.time_increment = residualErrors_.time_increment;
    alignedScan.scan_time = misalignedScan.scan_time = unknownScan.scan_time = residualErrors_.scan_time;
    int size = (int)residualErrors_.ranges.size();
    alignedScan.ranges.resize(size);
    misalignedScan.ranges.resize(size);
    unknownScan.ranges.resize(size);
    alignedScan.intensities.resize(size);
    misalignedScan.intensities.resize(size);
    unknownScan.intensities.resize(size);

    for (int i = 0; i < (int)usedResidualErrors_.size(); ++i)
    {
        int idx = usedScanIndices_[i];
        if (residualErrorClasses[i] == ALIGNED)
            alignedScan.ranges[idx] = residualErrors_.ranges[idx];
        else if (residualErrorClasses[i] == MISALIGNED)
            misalignedScan.ranges[idx] = residualErrors_.ranges[idx];
        else
            unknownScan.ranges[idx] = residualErrors_.ranges[idx];
    }
    alignedScanPub_.publish(alignedScan);
    misalignedScanPub_.publish(misalignedScan);
    unknownScanPub_.publish(unknownScan);
}