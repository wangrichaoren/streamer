#include "GeneralModels/MakeMat4fModel.hpp"

#include <QtCore/QDir>
#include <QtCore/QEvent>

#include "Widget/StreamerMainWindow.hpp"
#include <QJsonArray>
#include <QtWidgets/QFileDialog>

MakeMat4fModel::MakeMat4fModel()
    : _box(new QGroupBox())
    , _layout(new QVBoxLayout(_box))
{
    _box->setMinimumSize(200, 200);
    _box->setMaximumSize(500, 500);

    auto t_group = new QGroupBox("translate", _box);
    auto t_lay = new QHBoxLayout(_box);
    x = new QLineEdit("0", _box);
    y = new QLineEdit("0", _box);
    z = new QLineEdit("0", _box);
    auto xlay = new QHBoxLayout(_box);
    auto ylay = new QHBoxLayout(_box);
    auto zlay = new QHBoxLayout(_box);
    auto xlab = new QLabel("x", _box);
    auto ylab = new QLabel("y", _box);
    auto zlab = new QLabel("z", _box);
    createLineEditFormCurQObj(xlay, xlab, x);
    createLineEditFormCurQObj(ylay, ylab, y);
    createLineEditFormCurQObj(zlay, zlab, z);
    t_lay->addLayout(xlay);
    t_lay->addLayout(ylay);
    t_lay->addLayout(zlay);
    t_group->setLayout(t_lay);

    auto r_group = new QGroupBox("revolve", _box);
    auto r_lay = new QHBoxLayout(_box);
    rx = new QLineEdit("0", _box);
    ry = new QLineEdit("0", _box);
    rz = new QLineEdit("0", _box);
    auto rxlay = new QHBoxLayout(_box);
    auto rylay = new QHBoxLayout(_box);
    auto rzlay = new QHBoxLayout(_box);
    auto rxlab = new QLabel("rx", _box);
    auto rylab = new QLabel("ry", _box);
    auto rzlab = new QLabel("rz", _box);
    createLineEditFormCurQObj(rxlay, rxlab, rx);
    createLineEditFormCurQObj(rylay, rylab, ry);
    createLineEditFormCurQObj(rzlay, rzlab, rz);
    r_lay->addLayout(rxlay);
    r_lay->addLayout(rylay);
    r_lay->addLayout(rzlay);
    r_group->setLayout(r_lay);

    _layout->addWidget(t_group);
    _layout->addWidget(r_group);

    _box->setLayout(_layout);
    _box->resize(200, 200);
//    _box->installEventFilter(this);

    connect(x,&QLineEdit::editingFinished,[=]{compute();});
    connect(y,&QLineEdit::editingFinished,[=]{compute();});
    connect(z,&QLineEdit::editingFinished,[=]{compute();});
    connect(rx,&QLineEdit::editingFinished,[=]{compute();});
    connect(ry,&QLineEdit::editingFinished,[=]{compute();});
    connect(rz,&QLineEdit::editingFinished,[=]{compute();});
}

unsigned int MakeMat4fModel::nPorts(PortType portType) const
{
    unsigned int result = 1;

    switch (portType) {
    case PortType::In:
        result = 0;
        break;

    case PortType::Out:
        result = 2;
        break;

    default:
        break;
    }
    return result;
}

bool MakeMat4fModel::eventFilter(QObject *object, QEvent *event)
{
    return false;
}

NodeDataType MakeMat4fModel::dataType(PortType const portType, PortIndex const portIndex) const
{
    if (portType == PortType::Out) {
        if (portIndex == 0) {
            return Matrix4fData().type();
        } else if (portIndex == 1) {
            return ResultData().type();
        }
    } else {
        return NodeDataType();
    }
}

std::shared_ptr<NodeData> MakeMat4fModel::outData(PortIndex portIndex)
{
    if (portIndex == 0) {
        return std::make_shared<Matrix4fData>(_outMat4f);
    } else if (portIndex == 1) {
        return std::make_shared<ResultData>(_outRes);
    }
}

QJsonObject MakeMat4fModel::save() const
{
    auto saver = NodeDelegateModel::save();
    saver["x"] = x->text();
    saver["y"] = y->text();
    saver["z"] = z->text();

    saver["rx"] = rx->text();
    saver["ry"] = ry->text();
    saver["rz"] = rz->text();

    return saver;
}

void MakeMat4fModel::load(const QJsonObject &js)
{
    x->setText(js["x"].toString());
    y->setText(js["y"].toString());
    z->setText(js["z"].toString());

    rx->setText(js["rx"].toString());
    ry->setText(js["ry"].toString());
    rz->setText(js["rz"].toString());
}

void MakeMat4fModel::compute()
{
    bool f;
    auto xv = x->text().toFloat(&f);
    auto yv = y->text().toFloat(&f);
    auto zv = z->text().toFloat(&f);
    auto rxv = rx->text().toFloat(&f);
    auto ryv = ry->text().toFloat(&f);
    auto rzv = rz->text().toFloat(&f);
    if (!f) {
        return;
    }
    _outRes.clear();

    Eigen::AngleAxisf rotation_x(rxv, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf rotation_y(ryv, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf rotation_z(rzv, Eigen::Vector3f::UnitZ());

    Eigen::Translation3f translation(xv, yv, zv);

    _outMat4f << (translation * rotation_y * rotation_z * rotation_x).matrix();

    _outRes = "Transformation: \n" + std::to_string(_outMat4f(0, 0)) + " "
              + std::to_string(_outMat4f(0, 1)) + " " + std::to_string(_outMat4f(0, 2)) + " "
              + std::to_string(_outMat4f(0, 3)) + "\n" + std::to_string(_outMat4f(1, 0)) + " "
              + std::to_string(_outMat4f(1, 1)) + " " + std::to_string(_outMat4f(1, 2)) + " "
              + std::to_string(_outMat4f(1, 3)) + "\n" + std::to_string(_outMat4f(2, 0)) + " "
              + std::to_string(_outMat4f(2, 1)) + " " + std::to_string(_outMat4f(2, 2)) + " "
              + std::to_string(_outMat4f(2, 3)) + "\n" + std::to_string(_outMat4f(3, 0)) + " "
              + std::to_string(_outMat4f(3, 1)) + " " + std::to_string(_outMat4f(3, 2)) + " "
              + std::to_string(_outMat4f(3, 3)) + "\n";

    Q_EMIT dataUpdated(0);
    Q_EMIT dataUpdated(1);
}

void MakeMat4fModel::setInData(std::shared_ptr<NodeData> nodeData, PortIndex const portIndex){

};