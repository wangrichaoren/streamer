#pragma once

#include <iostream>
#include <QString>

#include <QtNodes/NodeData>

using QtNodes::NodeData;
using QtNodes::NodeDataType;

/// The class can potentially incapsulate any user data which
/// need to be transferred within the Node Editor graph
class StringData : public NodeData
{
public:
    StringData() {}

    StringData(QString const &string)
        : _str(string)
    {}

    NodeDataType type() const override
    {
        //       id      name
        return {"String Data", "String"};
    }

    std::string string() const { return _str.toStdString(); }
    QString qString() const { return _str; }

private:
    QString _str;
};
