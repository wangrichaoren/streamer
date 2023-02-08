#pragma once

#include <opencv2/opencv.hpp>

#include <QtNodes/NodeData>

//using QtNodes::NodeData;
//using QtNodes::NodeDataType;

/// The class can potentially incapsulate any user data which
/// need to be transferred within the Node Editor graph
class ImageData : public QtNodes::NodeData
{
public:
    ImageData() {}

    ImageData(const cv::Mat &m)
        : _mat(m)
    {}

    QtNodes::NodeDataType type() const override
    {
        //       id      name
        return {"cv_mat", "CVMat"};
    }

    cv::Mat mat() const { return _mat; }

private:
    cv::Mat _mat;
};
