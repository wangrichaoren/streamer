#include "CvBinaryModel.hpp"

#include "ImageData.hpp"

#include <QtCore/QDir>
#include <QtCore/QEvent>
#include <QtNodes/NodeDelegateModelRegistry>
#include <QtWidgets/QFileDialog>

CvBinaryModel::CvBinaryModel()
    : _label(new QLabel("Image Visual"))
    , _thresh_val(new QLineEdit("125"))
    , _max_val(new QLineEdit("255"))
    , _vlayout(new QVBoxLayout())
    , _box(new QGroupBox())
{
    auto bf = _box->font();
    bf.setBold(true);
    _box->setFont(bf);
    // 限制输入范围 -255~255
//    ^-?(255|([1,2]?[0-4]?\d|[1,2]?5[0-4]?)(\.\d)?)$
//    QRegExp regx("^[0-9]$|^[0-9]{2}$|^[1][0-9]{2}$|^[2][0-4][0-9]$|^[2][5][0-5]$");
    QRegExp regx("^-?(255|([1,2]?[0-4]?\\d|[1,2]?5[0-4]?)(\\.\\d)?)$");
    auto p_reg = new QRegExpValidator(regx, this);

    // show label
    _label->setAlignment(Qt::AlignVCenter | Qt::AlignHCenter);
    QFont f = _label->font();
    f.setItalic(true);
    _label->setFont(f);
    _label->setMinimumSize(200, 200);
    _label->installEventFilter(this);

    // 限制输入类型和范围
    _thresh_val->setValidator(p_reg);
    _max_val->setValidator(p_reg);

    // 取反
    auto inv_G = new QGroupBox("取反");
    auto inv_Hlay = new QHBoxLayout();
    _not_inv_r = new QRadioButton("否");
    _inv_r = new QRadioButton("是");
    inv_Hlay->addWidget(_not_inv_r);
    inv_Hlay->addWidget(_inv_r);
    inv_G->setLayout(inv_Hlay);
    _not_inv_r->setChecked(true);
    //    _not_inv_r->installEventFilter(this);
    QObject::connect(_not_inv_r, &QRadioButton::clicked, [=] { this->compute(); });
    QObject::connect(_inv_r, &QRadioButton::clicked, [=] { this->compute(); });

    // 单选
    auto type_G = new QGroupBox("方法");
    auto radio_VLay = new QVBoxLayout();
    auto radio_HLay1 = new QHBoxLayout();
    auto radio_HLay2 = new QHBoxLayout();
    _tozero_r = new QRadioButton("TOZERO");
    _trunc_r = new QRadioButton("TRUNC");
    _ostu_r = new QRadioButton("OSTU");
    _binary_r = new QRadioButton("BINARY");
    _binary_r->setChecked(true);
    radio_HLay1->addWidget(_binary_r);
    radio_HLay1->addWidget(_tozero_r);
    radio_HLay2->addWidget(_trunc_r);
    radio_HLay2->addWidget(_ostu_r);
    radio_VLay->addLayout(radio_HLay1);
    radio_VLay->addLayout(radio_HLay2);
    type_G->setLayout(radio_VLay);
    for (auto t : std::vector<QRadioButton *>{_tozero_r, _trunc_r, _ostu_r, _binary_r}) {
        QObject::connect(t, &QRadioButton::clicked, [=] { this->compute(); });
    }

    auto th_lay = new QHBoxLayout();
    auto th_lab = new QLabel("阈值    ");
    createLineEditFormCurQObj(th_lay, th_lab, _thresh_val);

    auto max_lay = new QHBoxLayout();
    auto max_lab = new QLabel("最大值");
    createLineEditFormCurQObj(max_lay, max_lab, _max_val);

    QObject::connect(_thresh_val, &QLineEdit::textChanged, [=] { this->compute(); });
    QObject::connect(_max_val, &QLineEdit::textChanged, [=] { this->compute(); });

    _vlayout->addWidget(_label);
    _vlayout->addWidget(inv_G);
    _vlayout->addWidget(type_G);
    _vlayout->addLayout(th_lay);
    _vlayout->addLayout(max_lay);

    _box->setLayout(_vlayout);
}

unsigned int CvBinaryModel::nPorts(PortType portType) const
{
    unsigned int result = 1;

    switch (portType) {
    case PortType::In:
        result = 1;
        break;

    case PortType::Out:
        result = 1;

    default:
        break;
    };

    return result;
}

bool CvBinaryModel::eventFilter(QObject *object, QEvent *event)
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

NodeDataType CvBinaryModel::dataType(PortType const, PortIndex const) const
{
    return ImageData().type();
}

std::shared_ptr<NodeData> CvBinaryModel::outData(PortIndex)
{
    // todo 不能返回原来的了
    return std::make_shared<ImageData>(_mat);
}

void CvBinaryModel::setInData(std::shared_ptr<NodeData> nodeData, PortIndex const)
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
void CvBinaryModel::compute()
{
    auto d = std::dynamic_pointer_cast<ImageData>(_nodeData);
    if (!d) {
        return;
    }
    if (d->mat().empty()) {
        return;
    }
    int type;
    if (_inv_r->isChecked()) {
        if (_binary_r->isCheckable()) {
            type = cv::THRESH_BINARY_INV;
        } else if (_tozero_r->isCheckable()) {
            type = cv::THRESH_TOZERO_INV;
        } else if (_trunc_r->isCheckable()) {
            _inv_r->setEnabled(false);
            type = cv::THRESH_TRUNC;
        } else if (_ostu_r->isCheckable()) {
            _inv_r->setEnabled(false);
            type = cv::THRESH_OTSU;
        }
    } else if (_not_inv_r->isChecked()) {
        if (_binary_r->isCheckable()) {
            type = cv::THRESH_BINARY;
        } else if (_tozero_r->isCheckable()) {
            type = cv::THRESH_TOZERO;
        } else if (_trunc_r->isCheckable()) {
            type = cv::THRESH_TRUNC;
        } else if (_ostu_r->isCheckable()) {
            type = cv::THRESH_OTSU;
        }
    }

    cv::threshold(d->mat(), _mat, _thresh_val->text().toDouble(), _max_val->text().toDouble(), type);

    _inv_r->setEnabled(true);
    _not_inv_r->setEnabled(true);

    int w = _label->width();
    int h = _label->height();

    auto pix = QPixmap::fromImage(cvMat2QImage(_mat));
    _label->setPixmap(pix.scaled(w, h, Qt::KeepAspectRatio));

    Q_EMIT dataUpdated(0);
}
