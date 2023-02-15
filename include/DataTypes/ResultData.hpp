#pragma once

#include <iostream>

#include <QtNodes/NodeData>

using QtNodes::NodeData;
using QtNodes::NodeDataType;

/// The class can potentially incapsulate any user data which
/// need to be transferred within the Node Editor graph
class ResultData : public NodeData
{
public:
    ResultData() {}

    ResultData(std::string const &res)
        : _result(res)
    {}

    NodeDataType type() const override
    {
        //       id      name
        return {"Result", "result"};
    }

    std::string getData() const { return _result; }

private:
    std::string _result;
};
