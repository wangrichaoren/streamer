#include "../include/CvModels/CvMorphModel.hpp"

#include "../include/DataTypes/ImageData.hpp"

#include "Widget/Full2DDialog.h"

#include <QtCore/QDir>
#include <QtCore/QEvent>
#include <QtNodes/NodeDelegateModelRegistry>
#include <QtWidgets/QFileDialog>

CvMorphModel::CvMorphModel()
    : _box(new QGroupBox())
    , _label(new QLabel("Image Visual", _box))
{
    auto full_lay = new QVBoxLayout(_box);
    auto f = _box->font();
    f.setBold(true);
    _box->setFont(f);

    _label->setAlignment(Qt::AlignVCenter | Qt::AlignHCenter);
    _label->setMinimumSize(200, 200);
    _label->installEventFilter(this);

    auto func_group = new QGroupBox("方法", _box);

    _dilation_R = new QRadioButton("膨胀", func_group);
    _erosion_R = new QRadioButton("腐蚀", func_group);
    _open_R = new QRadioButton("开操作", func_group);
    _close_R = new QRadioButton("闭操作", func_group);
    connect(_dilation_R, &QRadioButton::clicked, [=] { compute(); });
    connect(_erosion_R, &QRadioButton::clicked, [=] { compute(); });
    connect(_open_R, &QRadioButton::clicked, [=] { compute(); });
    connect(_close_R, &QRadioButton::clicked, [=] { compute(); });
    _dilation_R->setChecked(true);
    auto func_lay1 = new QHBoxLayout(_box);
    auto func_lay2 = new QHBoxLayout(_box);
    func_lay1->addWidget(_dilation_R);
    func_lay1->addWidget(_erosion_R);
    func_lay2->addWidget(_open_R);
    func_lay2->addWidget(_close_R);
    auto func_vlay = new QVBoxLayout(_box);
    func_vlay->addLayout(func_lay1);
    func_vlay->addLayout(func_lay2);
    func_group->setLayout(func_vlay);

    auto kernel_group = new QGroupBox("卷积核形态", _box);
    auto kernel_vlay = new QVBoxLayout(_box);
    _cross_R = new QRadioButton("十字形", _box);
    _ellipse_R = new QRadioButton("椭圆形", _box);
    connect(_cross_R, &QRadioButton::clicked, [=] { compute(); });
    connect(_ellipse_R, &QRadioButton::clicked, [=] { compute(); });
    _cross_R->setChecked(true);
    auto shape_lay = new QHBoxLayout(_box);
    shape_lay->addWidget(_cross_R);
    shape_lay->addWidget(_ellipse_R);
    kernel_vlay->addLayout(shape_lay);

    auto row_lay = new QHBoxLayout(_box);
    auto col_lay = new QHBoxLayout(_box);
    auto row_lab = new QLabel("宽", _box);
    auto col_lab = new QLabel("高", _box);
    _row_edit = new QLineEdit("5", _box);
    _col_edit = new QLineEdit("5", _box);
    connect(_row_edit, &QLineEdit::editingFinished, [=] { compute(); });
    connect(_col_edit, &QLineEdit::editingFinished, [=] { compute(); });

    createLineEditFormCurQObj(row_lay, row_lab, _row_edit);
    createLineEditFormCurQObj(col_lay, col_lab, _col_edit);
    kernel_vlay->addLayout(row_lay);
    kernel_vlay->addLayout(col_lay);
    kernel_group->setLayout(kernel_vlay);

    full_lay->addWidget(_label);
    full_lay->addWidget(func_group);
    full_lay->addWidget(kernel_group);
    _box->setLayout(full_lay);
}

unsigned int CvMorphModel::nPorts(PortType portType) const
{
    unsigned int result = 1;

    switch (portType) {
    case PortType::In:
        result = 1;
        break;

    case PortType::Out:
        result = 1;

    default:;
        break;
    };

    return result;
}

bool CvMorphModel::eventFilter(QObject *object, QEvent *event)
{
    if (object == _label) {
        int w = _label->width();
        int h = _label->height();

        if (event->type() == QEvent::Resize) {
            auto d = std::dynamic_pointer_cast<ImageData>(_nodeData);
            if (d) {
                if (_mat.empty()) {
                    return false;
                };
                auto pix = QPixmap::fromImage(cvMat2QImage(_mat));
                _label->setPixmap(pix.scaled(w, h, Qt::KeepAspectRatio));
            }
        } else if (event->type() == QEvent::MouseButtonPress) {
            if (_mat.empty()) {
                return false;
            }
            auto shower = new Full2DDialog(nullptr, &_mat);
            shower->setWindowFlags(Qt::Window | Qt::WindowStaysOnTopHint);
            shower->showNormal();
            shower->exec();
            shower->deleteLater();
        }
    }
    return false;
}

NodeDataType CvMorphModel::dataType(PortType const, PortIndex const) const
{
    return ImageData().type();
}

std::shared_ptr<NodeData> CvMorphModel::outData(PortIndex)
{
    return std::make_shared<ImageData>(_mat);
}

void CvMorphModel::setInData(std::shared_ptr<NodeData> nodeData, PortIndex const)
{
    _nodeData = nodeData;

    if (_nodeData) {
        auto d = std::dynamic_pointer_cast<ImageData>(_nodeData);
        if (d->mat().empty()) {
            return;
        };

        this->compute();

    } else {
        _label->setPixmap(QPixmap());
    }

    // dataUpdated 触发下游的节点更新
    Q_EMIT dataUpdated(0);
}
void CvMorphModel::compute()
{
    auto d = std::dynamic_pointer_cast<ImageData>(_nodeData);
    if (!d) {
        return;
    }
    if (d->mat().empty()) {
        return;
    }

    int w = _label->width();
    int h = _label->height();

    int shape;
    if (_cross_R->isChecked()) {
        shape = cv::MORPH_CROSS;
    } else if (_ellipse_R->isChecked()) {
        shape = cv::MORPH_ELLIPSE;
    }

    auto kernel = cv::getStructuringElement(shape,
                                            cv::Size(_row_edit->text().toInt(),
                                                     _col_edit->text().toInt()));
    auto src_mat = d->mat();
    if (_dilation_R->isChecked()) {
        cv::dilate(src_mat, _mat, kernel);
    } else if (_erosion_R->isChecked()) {
        cv::erode(src_mat, _mat, kernel);
    } else if (_open_R->isChecked()) {
        morphologyEx(src_mat, _mat, cv::MORPH_OPEN, kernel);
    } else if (_close_R->isChecked()) {
        morphologyEx(src_mat, _mat, cv::MORPH_CLOSE, kernel);
    }

    auto pix = QPixmap::fromImage(cvMat2QImage(_mat));
    _label->setPixmap(pix.scaled(w, h, Qt::KeepAspectRatio));

    Q_EMIT dataUpdated(0);
}
void CvMorphModel::load(const QJsonObject &s)
{
    _row_edit->setText(s["w"].toString());
    _col_edit->setText(s["h"].toString());

    _cross_R->setChecked(s["is_cross"].toBool());
    _ellipse_R->setChecked(s["is_ellipse"].toBool());
    _dilation_R->setChecked(s["is_dilation"].toBool());
    _erosion_R->setChecked(s["is_erosion"].toBool());
    _open_R->setChecked(s["is_open"].toBool());
    _close_R->setChecked(s["is_close"].toBool());
}

QJsonObject CvMorphModel::save() const
{
    auto s = NodeDelegateModel::save();
    s["w"] = _row_edit->text();
    s["h"] = _col_edit->text();
    s["is_cross"] = _cross_R->isChecked();
    s["is_ellipse"] = _ellipse_R->isChecked();

    s["is_dilation"] = _dilation_R->isChecked();
    s["is_erosion"] = _erosion_R->isChecked();
    s["is_open"] = _open_R->isChecked();
    s["is_close"] = _close_R->isChecked();

    return s;
}
