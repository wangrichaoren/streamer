#pragma once

#include <opencv2/opencv.hpp>

#include <QtNodes/NodeData>

using QtNodes::NodeData;
using QtNodes::NodeDataType;

/// The class can potentially incapsulate any user data which
/// need to be transferred within the Node Editor graph
class ImageData : public NodeData
{
public:
    ImageData() {}

    ImageData(cv::Mat const &cv_mat)
        : _cv_mat(cv_mat)
    {}

    NodeDataType type() const override
    {
        //       id      name
        return {"Cv Mat", "CvMat"};
    }

    cv::Mat mat() const { return _cv_mat; }

    void set_mat(cv::Mat m) const { m.copyTo(_cv_mat); };

private:
    cv::Mat _cv_mat;
};
