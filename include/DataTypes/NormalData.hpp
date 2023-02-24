#pragma once

#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <QtNodes/NodeData>

using QtNodes::NodeData;
using QtNodes::NodeDataType;

/// The class can potentially incapsulate any user data which
/// need to be transferred within the Node Editor graph
class NormalData : public NodeData
{
public:
    NormalData() {}

    NormalData(pcl::PointCloud<pcl::Normal>::Ptr normal)
        : _normal(normal)
    {}

    NodeDataType type() const override
    {
        //       id      name
        return {"normal", "normal"};
    }

    pcl::PointCloud<pcl::Normal>::Ptr getData() const { return _normal; }

private:
    pcl::PointCloud<pcl::Normal>::Ptr _normal;
};
