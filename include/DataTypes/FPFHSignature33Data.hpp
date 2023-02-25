#pragma once

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <QtNodes/NodeData>

using QtNodes::NodeData;
using QtNodes::NodeDataType;

/// The class can potentially incapsulate any user data which
/// need to be transferred within the Node Editor graph
class FPFHSignature33Data : public NodeData
{
public:
    FPFHSignature33Data() {}

    FPFHSignature33Data(pcl::PointCloud<pcl::FPFHSignature33>::Ptr feature33)
        : _feature33(feature33)
    {}

    NodeDataType type() const override
    {
        //       id      name
        return {"fpfh33", "fpfh33"};
    }

    pcl::PointCloud<pcl::FPFHSignature33>::Ptr getData() const { return _feature33; }

private:
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr _feature33;
};
