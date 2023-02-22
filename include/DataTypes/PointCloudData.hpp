#pragma once

#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <QtNodes/NodeData>

using QtNodes::NodeData;
using QtNodes::NodeDataType;

/// The class can potentially incapsulate any user data which
/// need to be transferred within the Node Editor graph
class PointCloudData : public NodeData
{
public:
    PointCloudData() {}

    PointCloudData(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc)
        : _pc(pc)
    {}

    NodeDataType type() const override
    {
        //       id      name
        return {"pc", "pc"};
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getData() const { return _pc; }

private:
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr _pc;
};
