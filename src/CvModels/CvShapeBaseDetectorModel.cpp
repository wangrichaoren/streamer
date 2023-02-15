#include "CvModels/CvShapeBaseDetectorModel.hpp"

#include "DataTypes/ImageData.hpp"

#include <QtCore/QDir>
#include <QtCore/QEvent>
#include <QtNodes/NodeDelegateModelRegistry>
#include <QtWidgets/QFileDialog>

CvShapeBaseDetectorModel::CvShapeBaseDetectorModel()
    : _box(new QGroupBox())
    , _label(new QLabel("Image Visual", _box))
    , all_lay(new QVBoxLayout(_box))
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

    // todo
    path_show = new QLineEdit(_box);
    path_show->setEnabled(false);
    auto load_btn = new QPushButton("加载模板", _box);
    auto threshold_group = new QGroupBox("阈值");
    auto threshold_lay = new QVBoxLayout();
    threshold_edit = new QLineEdit("90");
    threshold_lay->addWidget(threshold_edit);
    threshold_group->setLayout(threshold_lay);

    all_lay->addWidget(_label);
    all_lay->addWidget(path_show);
    all_lay->addWidget(load_btn);
    all_lay->addWidget(threshold_group);
    // todo
    _box->setLayout(all_lay);
    _box->resize(200, 200);

    connect(path_show, &QLineEdit::textChanged, [=](const QString &s) {
        if (!s.isEmpty()) {
            threshold_edit->setEnabled(true);
            compute();
        } else {
            threshold_edit->setEnabled(false);
        }
    });

    connect(threshold_edit, &QLineEdit::editingFinished, [=] { compute(); });

    connect(load_btn, &QPushButton::clicked, [=] {
        // choose dir & check file if full
        auto dirURL = QFileDialog::getExistingDirectoryUrl(nullptr,
                                                           tr("Choose Template Directory"),
                                                           QDir::homePath());
        auto dirName = dirURL.path();
        if (dirName.isEmpty()) {
            return;
        }
        std::vector<std::string> files;
        auto f = getDirectoryFile(dirName.toStdString(), files);
        if (f) {
            if (files.empty()) {
                createMessageBox(nullptr,
                                 ":icons/error.png",
                                 "加载错误",
                                 "当前文件夹下不存在可用于匹配的模板文件",
                                 1,
                                 {"返回"});
                path_show->clear();
                return;
            }
            bool is_template = false;
            bool is_info = false;
            bool is_img = false;

            for (const auto &file : files) {
                if (file == (dirName.toStdString() + "/template.yaml")) {
                    is_template = true;
                }
                if (file == (dirName.toStdString() + "/info.yaml")) {
                    is_info = true;
                }
                if (file == (dirName.toStdString() + "/template.png")) {
                    is_img = true;
                }
            }

            if ((is_template + is_info + is_img) != 3) {
                createMessageBox(nullptr,
                                 ":icons/error.png",
                                 "加载错误",
                                 "当前文件夹下不存在可用于匹配的模板文件",
                                 1,
                                 {"返回"});
                path_show->clear();
                return;
            }
            // check ok
            path_show->setText(dirName);
            //            compute();

        } else {
            createMessageBox(nullptr,
                             ":icons/error.png",
                             "加载错误",
                             "当前文件夹下不存在可用于匹配的模板文件",
                             1,
                             {"返回"});
            path_show->clear();
            return;
        }
    });

    path_show->textChanged("");
}

unsigned int CvShapeBaseDetectorModel::nPorts(PortType portType) const
{
    unsigned int result = 1;

    switch (portType) {
    case PortType::In:
        result = 1;
        break;

    case PortType::Out:
        result = 2;

    default:
        break;
    };

    return result;
}

bool CvShapeBaseDetectorModel::eventFilter(QObject *object, QEvent *event)
{
    if (object == _label) {
        this->embeddedWidget()->setCursor(Qt::PointingHandCursor);
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

NodeDataType CvShapeBaseDetectorModel::dataType(PortType const portType,
                                                PortIndex const portIndex) const
{
    switch (portType) {
    case PortType::In:
        return ImageData().type();
    case PortType::Out:
        if (portIndex == 0) {
            return ImageData().type();
        } else if (portIndex == 1) {
            return ResultData().type();
        }
    case PortType::None:
        break;
    }
    return NodeDataType();
}

std::shared_ptr<NodeData> CvShapeBaseDetectorModel::outData(PortIndex)
{
    return std::make_shared<ImageData>(_mat);
}

void CvShapeBaseDetectorModel::setInData(std::shared_ptr<NodeData> nodeData, PortIndex const)
{
    _nodeData = nodeData;

    if (_nodeData) {
        auto d = std::dynamic_pointer_cast<ImageData>(_nodeData);
        if (d->mat().empty()) {
            return;
        };
        if (path_show->text().isEmpty()) {
            return;
        }

        this->compute();

    } else {
        _label->setPixmap(QPixmap());
    }

    // dataUpdated 触发下游的节点更新
    Q_EMIT dataUpdated(0);
}
void CvShapeBaseDetectorModel::compute()
{
    auto d = std::dynamic_pointer_cast<ImageData>(_nodeData);
    if (!d) {
        return;
    }

    if (d->mat().empty()) {
        return;
    }

    if (path_show->text().isEmpty()) {
        return;
    }

    // todo ----
    line2Dup::Detector detector({4, 8}); // T 必须偶数

    std::vector<std::string> ids;
    ids.push_back("test");

    auto template_path = path_show->text().toStdString() + "/template.yaml";
    detector.readClasses(ids, template_path);

    // angle & scale are saved here, fetched by match id
    auto info_path = path_show->text().toStdString() + "/info.yaml";
    auto infos = shape_based_matching::shapeInfo_producer::load_infos(info_path);
    auto tmp_img = cv::imread(path_show->text().toStdString() + "/template.png");
    auto tmp_h = tmp_img.rows;
    auto tmp_w = tmp_img.cols;

    cv::Mat test_img = d->mat();
    // 4 channel -> 3
    cv::cvtColor(test_img, test_img, cv::COLOR_RGBA2RGB);

    int padding = 250;
    cv::Mat padded_img = cv::Mat(test_img.rows + 2 * padding,
                                 test_img.cols + 2 * padding,
                                 test_img.type(),
                                 cv::Scalar::all(0));
    test_img.copyTo(padded_img(cv::Rect(padding, padding, test_img.cols, test_img.rows)));

    int stride = 16;
    int n = padded_img.rows / stride;
    int m = padded_img.cols / stride;
    cv::Rect roi(0, 0, stride * m, stride * n);
    _mat = padded_img(roi).clone();
    assert(img.isContinuous());

    auto matches = detector.match(_mat, 90, ids);

    if (_mat.channels() == 1)
        cvtColor(_mat, _mat, CV_GRAY2BGR);

    //    std::cout << "matches.size(): " << matches.size() << std::endl;
    size_t top5 = 1;
    if (top5 > matches.size())
        top5 = matches.size();
    for (size_t i = 0; i < top5; i++) {
        auto match = matches[i];
        auto templ = detector.getTemplates("test", match.template_id);

        // 270 is width of template image
        // 100 is padding when training
        // tl_x/y: template croping topleft corner when training

        float r_scaled = tmp_w / 2.0f * infos[match.template_id].scale;

        // scaling won't affect this, because it has been determined by warpAffine
        // cv::warpAffine(src, dst, rot_mat, src.size()); last param
        float train_img_half_width = tmp_w / 2.0f + 100;
        float train_img_half_height = tmp_h / 2.0f + 100;

        // center x,y of train_img in test img
        float x = match.x - templ[0].tl_x + train_img_half_width;
        float y = match.y - templ[0].tl_y + train_img_half_height;

        cv::Vec3b randColor;
        randColor[0] = rand() % 155 + 100;
        randColor[1] = rand() % 155 + 100;
        randColor[2] = rand() % 155 + 100;
        for (int i = 0; i < templ[0].features.size(); i++) {
            auto feat = templ[0].features[i];
            cv::circle(_mat, {feat.x + match.x, feat.y + match.y}, 3, randColor, -1);
        }

        cv::putText(_mat,
                    std::to_string(int(round(match.similarity))),
                    cv::Point(match.x + r_scaled - 10, match.y - 3),
                    cv::FONT_HERSHEY_PLAIN,
                    2,
                    randColor);

        cv::RotatedRect rotatedRectangle({x, y},
                                         {2 * r_scaled, 2 * r_scaled},
                                         -infos[match.template_id].angle);

        cv::Point2f vertices[4];
        rotatedRectangle.points(vertices);
        for (int i = 0; i < 4; i++) {
            int next = (i + 1 == 4) ? 0 : (i + 1);
            cv::line(_mat, vertices[i], vertices[next], randColor, 2);
        }

        //        std::cout << "\nmatch.template_id: " << match.template_id << std::endl;
        //        std::cout << "match.similarity: " << match.similarity << std::endl;
    }
    // todo ----

    int w = _label->width();
    int h = _label->height();

    auto pix = QPixmap::fromImage(cvMat2QImage(_mat));
    _label->setPixmap(pix.scaled(w, h, Qt::KeepAspectRatio));

    Q_EMIT dataUpdated(0);
    //    Q_EMIT dataUpdated(1);
}
void CvShapeBaseDetectorModel::load(const QJsonObject &s)
{
    path_show->setText(s["path"].toString());
    threshold_edit->setText(s["threshold"].toString());
    threshold_edit->textChanged(s["threshold"].toString());
}

QJsonObject CvShapeBaseDetectorModel::save() const
{
    auto s = NodeDelegateModel::save();
    s["path"] = path_show->text();
    s["threshold"] = threshold_edit->text();
    return s;
}