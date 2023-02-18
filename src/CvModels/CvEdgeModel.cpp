#include "../include/CvModels/CvEdgeModel.hpp"

#include "../include/DataTypes/ImageData.hpp"

#include "Widget/Full2DDialog.h"

#include <QtCore/QDir>
#include <QtCore/QEvent>
#include <QtNodes/NodeDelegateModelRegistry>
#include <QtWidgets/QFileDialog>

CvEdgeModel::CvEdgeModel()
    : _box(new QGroupBox())
    , _label(new QLabel("Image Visual", _box))
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
    auto type_group = new QGroupBox("方法", _box);

    auto type_hlay = new QHBoxLayout(_box);
    canny_radio = new QRadioButton("canny", _box);
    laplacian_radio = new QRadioButton("laplacian", _box);
    sobel_radio = new QRadioButton("sobel", _box);
    type_hlay->addWidget(canny_radio);
    type_hlay->addWidget(laplacian_radio);
    type_hlay->addWidget(sobel_radio);
    type_group->setLayout(type_hlay);

    // ----
    createCannyGroupBox();
    createLaplacianGroupBox();
    createSobelGroupBox();
    //----

    all_lay = new QVBoxLayout(_box);
    all_lay->addWidget(_label);
    all_lay->addWidget(type_group);

    _box->resize(200, 200);

    connect(canny_radio, &QRadioButton::clicked, [=] {
        sobel_group->hide();
        laplacian_group->hide();
        canny_group->show();
        all_lay->addWidget(canny_group);
        _box->setLayout(all_lay);
        compute();
    });

    connect(laplacian_radio, &QRadioButton::clicked, [=] {
        sobel_group->hide();
        canny_group->hide();
        laplacian_group->show();
        all_lay->addWidget(laplacian_group);
        _box->setLayout(all_lay);
        compute();
    });

    connect(sobel_radio, &QRadioButton::clicked, [=] {
        laplacian_group->hide();
        canny_group->hide();
        sobel_group->show();
        all_lay->addWidget(sobel_group);
        _box->setLayout(all_lay);
        compute();
    });

    connect(laplacian_radio, &QRadioButton::objectNameChanged, [=] { compute(); });
    connect(sobel_radio, &QRadioButton::objectNameChanged, [=] { compute(); });
    connect(canny_radio, &QRadioButton::objectNameChanged, [=] { compute(); });
    connect(l2gradient_canny_true, &QRadioButton::objectNameChanged, [=] { compute(); });
    connect(l2gradient_canny_false, &QRadioButton::objectNameChanged, [=] { compute(); });

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

    connect(threshold1_canny, &QLineEdit::editingFinished, [=] { compute(); });
    connect(threshold2_canny, &QLineEdit::editingFinished, [=] { compute(); });
    connect(aperture_size_canny, &QLineEdit::editingFinished, [=] { compute(); });

    canny_radio->setChecked(true);
    Q_EMIT canny_radio->clicked(true);
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
    } else if (canny_radio->isChecked()) {
        auto th1 = threshold1_canny->text().toDouble();
        auto th2 = threshold2_canny->text().toDouble();
        auto as = aperture_size_canny->text().toInt();

        if (as < 3) {
            as = 3;
            aperture_size_canny->setText(std::to_string(as).c_str());
        }
        if (as > 7) {
            as = 7;
            aperture_size_canny->setText(std::to_string(as).c_str());
        }

        bool f;
        if (l2gradient_canny_true->isChecked()) {
            f = true;
        } else {
            f = false;
        }
        cv::Canny(d->mat(), _mat, th1, th2, as, f);
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

    canny_radio->setChecked(s["canny_radio"].toBool());
    l2gradient_canny_true->setChecked(s["l2gradient_canny_true"].toBool());
    l2gradient_canny_false->setChecked(s["l2gradient_canny_false"].toBool());
    threshold1_canny->setText(s["threshold1_canny"].toString());
    threshold2_canny->setText(s["threshold2_canny"].toString());
    aperture_size_canny->setText(s["aperture_size_canny"].toString());

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
    } else if (sobel_radio->isChecked()) {
        emit sobel_radio->clicked(true);
    } else if (canny_radio->isChecked()) {
        emit canny_radio->clicked(true);
    }
}

QJsonObject CvEdgeModel::save() const
{
    auto s = NodeDelegateModel::save();
    s["canny_radio"] = canny_radio->isChecked();
    s["threshold1_canny"] = threshold1_canny->text();
    s["threshold2_canny"] = threshold2_canny->text();
    s["aperture_size_canny"] = aperture_size_canny->text();
    s["l2gradient_canny_true"] = l2gradient_canny_true->isChecked();
    s["l2gradient_canny_false"] = l2gradient_canny_false->isChecked();

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
    sobel_group = new QGroupBox(_box);
    auto sbl_vlay = new QVBoxLayout(_box);

    auto sbl_hlay1 = new QHBoxLayout(_box);
    auto sbl_hlay2 = new QHBoxLayout(_box);
    auto sbl_hlay3 = new QHBoxLayout(_box);
    auto sbl_hlay4 = new QHBoxLayout(_box);
    auto sbl_hlay5 = new QHBoxLayout(_box);
    auto sbl_hlay6 = new QHBoxLayout(_box);

    auto dep_slab = new QLabel("depth", _box);
    auto size_slab = new QLabel("ksize", _box);
    auto dx_slab = new QLabel("dx       ", _box);
    auto dy_slab = new QLabel("dy      ", _box);
    auto scale_slab = new QLabel("scale  ", _box);
    auto delta_slab = new QLabel("delta ", _box);

    ddepth_sobel = new QLineEdit("-1", _box);
    ksize_sobel = new QLineEdit("3", _box);
    dx_sobel = new QLineEdit("1", _box);
    dy_sobel = new QLineEdit("1", _box);
    scale_sobel = new QLineEdit("1", _box);
    delta_sobel = new QLineEdit("0", _box);

    createLineEditFormCurQObj(sbl_hlay1, dep_slab, ddepth_sobel);
    createLineEditFormCurQObj(sbl_hlay2, size_slab, ksize_sobel);
    createLineEditFormCurQObj(sbl_hlay3, dx_slab, dx_sobel);
    createLineEditFormCurQObj(sbl_hlay4, dy_slab, dy_sobel);
    createLineEditFormCurQObj(sbl_hlay5, scale_slab, scale_sobel);
    createLineEditFormCurQObj(sbl_hlay6, delta_slab, delta_sobel);

    auto kv_lay1 = new QHBoxLayout(_box);
    auto kv_lay2 = new QHBoxLayout(_box);
    auto kv_lay3 = new QHBoxLayout(_box);
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

void CvEdgeModel::createCannyGroupBox()
{
    //    cv::Canny()
    canny_group = new QGroupBox(_box);

    auto canny_vlay = new QVBoxLayout(_box);

    auto canny_hlay1 = new QHBoxLayout(_box);
    auto canny_hlay2 = new QHBoxLayout(_box);
    auto canny_hlay3 = new QHBoxLayout(_box);
    auto canny_hlay4 = new QHBoxLayout(_box);
    auto canny_hlay5 = new QHBoxLayout(_box);

    auto threshold1_lab = new QLabel("threshold1", _box);
    auto threshold2_lab = new QLabel("threshold2", _box);
    auto aperture_size_slab = new QLabel("apertureSize", _box);

    threshold1_canny = new QLineEdit("50", _box);
    threshold2_canny = new QLineEdit("100", _box);
    aperture_size_canny = new QLineEdit("3", _box);

    createLineEditFormCurQObj(canny_hlay1, threshold1_lab, threshold1_canny);
    createLineEditFormCurQObj(canny_hlay2, threshold2_lab, threshold2_canny);
    createLineEditFormCurQObj(canny_hlay3, aperture_size_slab, aperture_size_canny);

    auto iASG = new QGroupBox("isApertureSize", _box);
    auto iASG_lay = new QVBoxLayout(_box);
    l2gradient_canny_false = new QRadioButton("false", _box);
    l2gradient_canny_true = new QRadioButton("true", _box);
    auto iasg_hlay = new QHBoxLayout(_box);
    iasg_hlay->addWidget(l2gradient_canny_false);
    iasg_hlay->addWidget(l2gradient_canny_true);
    iASG_lay->addLayout(iasg_hlay);
    l2gradient_canny_false->setChecked(true);
    iASG->setLayout(iASG_lay);

    canny_vlay->addLayout(canny_hlay1);
    canny_vlay->addLayout(canny_hlay2);
    canny_vlay->addLayout(canny_hlay3);
    canny_vlay->addWidget(iASG);
    canny_group->setLayout(canny_vlay);
}
