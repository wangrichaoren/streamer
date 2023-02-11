#include "../include/CvModels/CvMorphModel.hpp"

#include "../include/DataTypes/ImageData.hpp"

#include <QtCore/QDir>
#include <QtCore/QEvent>
#include <QtNodes/NodeDelegateModelRegistry>
#include <QtWidgets/QFileDialog>

CvMorphModel::CvMorphModel()
    : _label(new QLabel("Image Visual"))
    , _box(new QGroupBox())
{
    auto f = _box->font();
    f.setBold(true);
    _box->setFont(f);

    _label->setAlignment(Qt::AlignVCenter | Qt::AlignHCenter);
    _label->setMinimumSize(200, 200);
    _label->installEventFilter(this);

    auto func_group = new QGroupBox("方法");
    _dilation_R = new QRadioButton("膨胀");
    _erosion_R = new QRadioButton("腐蚀");
    _open_R = new QRadioButton("开操作");
    _close_R = new QRadioButton("闭操作");
    connect(_dilation_R, &QRadioButton::clicked, [=] { compute(); });
    connect(_erosion_R, &QRadioButton::clicked, [=] { compute(); });
    connect(_open_R, &QRadioButton::clicked, [=] { compute(); });
    connect(_close_R, &QRadioButton::clicked, [=] { compute(); });
    _dilation_R->setChecked(true);
    auto func_lay1 = new QHBoxLayout();
    auto func_lay2 = new QHBoxLayout();
    func_lay1->addWidget(_dilation_R);
    func_lay1->addWidget(_erosion_R);
    func_lay2->addWidget(_open_R);
    func_lay2->addWidget(_close_R);
    auto func_vlay = new QVBoxLayout();
    func_vlay->addLayout(func_lay1);
    func_vlay->addLayout(func_lay2);
    func_group->setLayout(func_vlay);

    auto kernel_group = new QGroupBox("卷积核形态");
    auto kernel_vlay = new QVBoxLayout();
    _cross_R = new QRadioButton("十字形");
    _ellipse_R = new QRadioButton("椭圆形");
    connect(_cross_R, &QRadioButton::clicked, [=] { compute(); });
    connect(_ellipse_R, &QRadioButton::clicked, [=] { compute(); });
    _cross_R->setChecked(true);
    auto shape_lay = new QHBoxLayout();
    shape_lay->addWidget(_cross_R);
    shape_lay->addWidget(_ellipse_R);
    kernel_vlay->addLayout(shape_lay);

    auto row_lay = new QHBoxLayout();
    auto col_lay = new QHBoxLayout();
    auto row_lab = new QLabel("宽");
    auto col_lab = new QLabel("高");
    _row_edit = new QLineEdit("5");
    _col_edit = new QLineEdit("5");
    connect(_row_edit, &QLineEdit::editingFinished, [=] { compute(); });
    connect(_col_edit, &QLineEdit::editingFinished, [=] { compute(); });

    createLineEditFormCurQObj(row_lay, row_lab, _row_edit);
    createLineEditFormCurQObj(col_lay, col_lab, _col_edit);
    kernel_vlay->addLayout(row_lay);
    kernel_vlay->addLayout(col_lay);
    kernel_group->setLayout(kernel_vlay);

    auto full_lay = new QVBoxLayout();
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
