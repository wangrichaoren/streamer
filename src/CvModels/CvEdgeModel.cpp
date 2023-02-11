#include "../include/CvModels/CvEdgeModel.hpp"

#include "../include/DataTypes/ImageData.hpp"

#include <QtCore/QDir>
#include <QtCore/QEvent>
#include <QtNodes/NodeDelegateModelRegistry>
#include <QtWidgets/QFileDialog>

CvEdgeModel::CvEdgeModel()
    : _label(new QLabel("Image Visual"))
    , _box(new QGroupBox())
    , laplacian_group{nullptr}
    , sobel_group{nullptr}
{
    auto bf = _box->font();
    bf.setBold(true);
    _box->setFont(bf);

    // show label
    _label->setAlignment(Qt::AlignVCenter | Qt::AlignHCenter);
    QFont f = _label->font();
    f.setItalic(true);
    _label->setFont(f);
    _label->setMinimumSize(200, 200);
    _label->installEventFilter(this);

    // radio group
    auto type_group = new QGroupBox("方法");

    auto type_hlay = new QHBoxLayout();
    laplacian_radio = new QRadioButton("laplacian");
    sobel_radio = new QRadioButton("sobel");
    type_hlay->addWidget(laplacian_radio);
    type_hlay->addWidget(sobel_radio);
    type_group->setLayout(type_hlay);

    // ----
    createLaplacianGroupBox();
    createSobelGroupBox();
    //----

    all_lay = new QVBoxLayout();
    all_lay->addWidget(_label);
    all_lay->addWidget(type_group);

    _box->resize(200, 200);

    connect(laplacian_radio, &QRadioButton::clicked, [=] {
        sobel_group->hide();
        laplacian_group->show();
        all_lay->addWidget(laplacian_group);
        _box->setLayout(all_lay);
        compute();
    });

    connect(sobel_radio, &QRadioButton::clicked, [=] {
        laplacian_group->hide();
        sobel_group->show();
        all_lay->addWidget(sobel_group);
        _box->setLayout(all_lay);
        compute();
    });

    connect(laplacian_radio, &QRadioButton::clicked, [=] { compute(); });
    connect(sobel_radio, &QRadioButton::clicked, [=] { compute(); });

    connect(ddepth_laplacian, &QLineEdit::editingFinished, [=] { compute(); });
    connect(ksize_laplacian, &QLineEdit::editingFinished, [=] { compute(); });
    connect(scale_laplacian, &QLineEdit::editingFinished, [=] { compute(); });
    connect(delta_laplacian, &QLineEdit::editingFinished, [=] { compute(); });
    connect(ddepth_sobel, &QLineEdit::editingFinished, [=] { compute(); });
    connect(ksize_sobel, &QLineEdit::editingFinished, [=] { compute(); });
    connect(dx_sobel, &QLineEdit::editingFinished, [=] { compute(); });
    connect(dy_sobel, &QLineEdit::editingFinished, [=] { compute(); });
    connect(scale_sobel, &QLineEdit::editingFinished, [=] { compute(); });
    connect(delta_sobel, &QLineEdit::editingFinished, [=] { compute(); });

    laplacian_radio->setChecked(true);
    Q_EMIT laplacian_radio->clicked(true);
}

unsigned int CvEdgeModel::nPorts(PortType portType) const
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

bool CvEdgeModel::eventFilter(QObject *object, QEvent *event)
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

NodeDataType CvEdgeModel::dataType(PortType const, PortIndex const) const
{
    return ImageData().type();
}

std::shared_ptr<NodeData> CvEdgeModel::outData(PortIndex)
{
    return std::make_shared<ImageData>(_mat);
}

void CvEdgeModel::setInData(std::shared_ptr<NodeData> nodeData, PortIndex const)
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
void CvEdgeModel::compute()
{
    auto d = std::dynamic_pointer_cast<ImageData>(_nodeData);
    if (!d) {
        return;
    }
    if (d->mat().empty()) {
        return;
    }

    if (laplacian_radio->isChecked()) {
        auto ddep = ddepth_laplacian->text().toInt();
        auto ksize = ksize_laplacian->text().toInt();
        auto scale = scale_laplacian->text().toInt();
        auto delta = delta_laplacian->text().toInt();

        if (ddep > 0) {
            ddep = -ddep;
            ddepth_laplacian->setText(std::to_string(ddep).c_str());
        }

        if (ksize % 2 == 0) {
            ksize += 1;
            ksize_laplacian->setText(std::to_string(ksize).c_str());
        }
        if (ksize >= 31) {
            ksize = 31;
            ksize_laplacian->setText(std::to_string(ksize).c_str());
        }

        cv::Laplacian(d->mat(), _mat, ddep, ksize, scale, delta);

    } else if (sobel_radio->isChecked()) {
        auto s_ddepth = ddepth_sobel->text().toInt();
        auto s_dx = dx_sobel->text().toInt();
        auto s_dy = dy_sobel->text().toInt();
        auto s_ksize = ksize_sobel->text().toInt();
        auto s_scale = scale_sobel->text().toInt();
        auto s_delta = delta_sobel->text().toInt();

        if (s_ddepth > 0) {
            s_ddepth = -s_ddepth;
            ddepth_sobel->setText(std::to_string(s_ddepth).c_str());
        }

//        if (s_ksize == 0) {
//            s_ksize = 1;
//            ksize_sobel->setText(std::to_string(s_ksize).c_str());
//        }

        if (s_ksize % 2 == 0) {
            s_ksize += 1;
            ksize_sobel->setText(std::to_string(s_ksize).c_str());
        }

        if (s_ksize <= 3) {
            s_ksize = 3;
            ksize_sobel->setText(std::to_string(s_ksize).c_str());
        }

        if (s_ksize >= 31) {
            s_ksize = 31;
            ksize_sobel->setText(std::to_string(s_ksize).c_str());
        }

        if (s_dx >= s_ksize) {
            s_dx = s_ksize - 1;
            dx_sobel->setText(std::to_string(s_dx).c_str());
        }

        if (s_dy >= s_ksize) {
            s_dy = s_ksize - 1;
            dy_sobel->setText(std::to_string(s_dy).c_str());
        }

        cv::Sobel(d->mat(), _mat, s_ddepth, s_dx, s_dy, s_ksize, s_scale, s_delta);
    }

    int w = _label->width();
    int h = _label->height();

    auto pix = QPixmap::fromImage(cvMat2QImage(_mat));
    _label->setPixmap(pix.scaled(w, h, Qt::KeepAspectRatio));

    Q_EMIT dataUpdated(0);
}
void CvEdgeModel::load(const QJsonObject &s)
{
    laplacian_radio->setChecked(s["laplacian_radio"].toBool());
    sobel_radio->setChecked(s["sobel_radio"].toBool());

    ddepth_laplacian->setText(s["ddepth_laplacian"].toString());
    ksize_laplacian->setText(s["ksize_laplacian"].toString());
    scale_laplacian->setText(s["scale_laplacian"].toString());
    delta_laplacian->setText(s["delta_laplacian"].toString());
    ddepth_sobel->setText(s["ddepth_sobel"].toString());
    ksize_sobel->setText(s["ksize_sobel"].toString());
    dx_sobel->setText(s["dx_sobel"].toString());
    dy_sobel->setText(s["dy_sobel"].toString());
    scale_sobel->setText(s["scale_sobel"].toString());
    delta_sobel->setText(s["delta_sobel"].toString());

    if (laplacian_radio->isChecked()) {
        emit laplacian_radio->clicked(true);
    } else {
        emit sobel_radio->clicked(true);
    }
}

QJsonObject CvEdgeModel::save() const
{
    auto s = NodeDelegateModel::save();
    s["laplacian_radio"] = laplacian_radio->isChecked();
    s["ddepth_laplacian"] = ddepth_laplacian->text();
    s["ksize_laplacian"] = ksize_laplacian->text();
    s["scale_laplacian"] = scale_laplacian->text();
    s["delta_laplacian"] = delta_laplacian->text();

    s["sobel_radio"] = sobel_radio->isChecked();
    s["ddepth_sobel"] = ddepth_sobel->text();
    s["ksize_sobel"] = ksize_sobel->text();
    s["dx_sobel"] = dx_sobel->text();
    s["dy_sobel"] = dy_sobel->text();
    s["scale_sobel"] = scale_sobel->text();
    s["delta_sobel"] = delta_sobel->text();

    return s;
}
void CvEdgeModel::createLaplacianGroupBox()
{
    // laplacian widget
    laplacian_group = new QGroupBox();
    auto lap_vlay = new QVBoxLayout();
    auto lpl_hlay1 = new QHBoxLayout();
    auto lpl_hlay2 = new QHBoxLayout();
    auto lpl_hlay3 = new QHBoxLayout();
    auto lpl_hlay4 = new QHBoxLayout();

    auto kv_hlay1 = new QHBoxLayout();
    auto kv_hlay2 = new QHBoxLayout();

    auto dep_lab = new QLabel("depth");
    auto size_lab = new QLabel("ksize");
    auto scale_lab = new QLabel("scale  ");
    auto delta_lab = new QLabel("delta");

    ddepth_laplacian = new QLineEdit("-1");
    ksize_laplacian = new QLineEdit("1");
    scale_laplacian = new QLineEdit("1");
    delta_laplacian = new QLineEdit("0");

    createLineEditFormCurQObj(lpl_hlay1, dep_lab, ddepth_laplacian);
    createLineEditFormCurQObj(lpl_hlay2, size_lab, ksize_laplacian);
    createLineEditFormCurQObj(lpl_hlay3, scale_lab, scale_laplacian);
    createLineEditFormCurQObj(lpl_hlay4, delta_lab, delta_laplacian);

    kv_hlay1->addLayout(lpl_hlay1);
    kv_hlay1->addLayout(lpl_hlay2);
    kv_hlay2->addLayout(lpl_hlay3);
    kv_hlay2->addLayout(lpl_hlay4);

    lap_vlay->addLayout(kv_hlay1);
    lap_vlay->addLayout(kv_hlay2);
    laplacian_group->setLayout(lap_vlay);
}

void CvEdgeModel::createSobelGroupBox()
{
    // sobel widget
    sobel_group = new QGroupBox();
    auto sbl_vlay = new QVBoxLayout();

    auto sbl_hlay1 = new QHBoxLayout();
    auto sbl_hlay2 = new QHBoxLayout();
    auto sbl_hlay3 = new QHBoxLayout();
    auto sbl_hlay4 = new QHBoxLayout();
    auto sbl_hlay5 = new QHBoxLayout();
    auto sbl_hlay6 = new QHBoxLayout();

    auto dep_slab = new QLabel("depth");
    auto size_slab = new QLabel("ksize");
    auto dx_slab = new QLabel("dx       ");
    auto dy_slab = new QLabel("dy      ");
    auto scale_slab = new QLabel("scale  ");
    auto delta_slab = new QLabel("delta ");

    ddepth_sobel = new QLineEdit("-1");
    ksize_sobel = new QLineEdit("3");
    dx_sobel = new QLineEdit("1");
    dy_sobel = new QLineEdit("1");
    scale_sobel = new QLineEdit("1");
    delta_sobel = new QLineEdit("0");

    createLineEditFormCurQObj(sbl_hlay1, dep_slab, ddepth_sobel);
    createLineEditFormCurQObj(sbl_hlay2, size_slab, ksize_sobel);
    createLineEditFormCurQObj(sbl_hlay3, dx_slab, dx_sobel);
    createLineEditFormCurQObj(sbl_hlay4, dy_slab, dy_sobel);
    createLineEditFormCurQObj(sbl_hlay5, scale_slab, scale_sobel);
    createLineEditFormCurQObj(sbl_hlay6, delta_slab, delta_sobel);

    auto kv_lay1 = new QHBoxLayout();
    auto kv_lay2 = new QHBoxLayout();
    auto kv_lay3 = new QHBoxLayout();
    kv_lay1->addLayout(sbl_hlay1);
    kv_lay1->addLayout(sbl_hlay2);
    kv_lay2->addLayout(sbl_hlay3);
    kv_lay2->addLayout(sbl_hlay4);
    kv_lay3->addLayout(sbl_hlay5);
    kv_lay3->addLayout(sbl_hlay6);
    sbl_vlay->addLayout(kv_lay1);
    sbl_vlay->addLayout(kv_lay2);
    sbl_vlay->addLayout(kv_lay3);
    sobel_group->setLayout(sbl_vlay);
}
