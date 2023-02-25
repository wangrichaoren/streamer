#pragma once

#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <QtNodes/NodeData>

using QtNodes::NodeData;
using QtNodes::NodeDataType;

/// The class can potentially incapsulate any user data which
/// need to be transferred within the Node Editor graph
class Matrix4fData : public NodeData
{
public:
    Matrix4fData() {}

    Matrix4fData(Eigen::Matrix4f mat4f)
        : _mat4f(mat4f)
    {}

    NodeDataType type() const override
    {
        //       id      name
        return {"mat4f", "mat4f"};
    }

    Eigen::Matrix4f getData() const { return _mat4f; }

private:
    Eigen::Matrix4f _mat4f;
};
