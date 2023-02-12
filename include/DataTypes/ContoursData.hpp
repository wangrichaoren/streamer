#pragma once

#include <opencv2/opencv.hpp>

#include <QtNodes/NodeData>

using QtNodes::NodeData;
using QtNodes::NodeDataType;

struct ContoursDataStruct
{
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierachy;
};

/// The class can potentially incapsulate any user data which
/// need to be transferred within the Node Editor graph
class ContoursData : public NodeData
{
public:
    ContoursData() {}

    ContoursData(ContoursDataStruct const &cds)
        : _cds(cds)
    {}

    NodeDataType type() const override
    {
        //       id      name
        return {"Contours", "contours"};
    }

    ContoursDataStruct getData() const { return _cds; }

private:
    ContoursDataStruct _cds;
};
