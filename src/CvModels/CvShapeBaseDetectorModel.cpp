#include "CvModels/CvShapeBaseDetectorModel.hpp"

#include "DataTypes/ImageData.hpp"

#include <QtCore/QDir>
#include <QtCore/QEvent>
#include <QtNodes/NodeDelegateModelRegistry>
#include <QtWidgets/QFileDialog>

// NMS, got from cv::dnn so we don't need opencv contrib
// just collapse it
namespace cv_dnn {
namespace {

template<typename T>
static inline bool SortScorePairDescend(const std::pair<float, T> &pair1,
                                        const std::pair<float, T> &pair2)
{
    return pair1.first > pair2.first;
}

} // namespace

inline void GetMaxScoreIndex(const std::vector<float> &scores,
                             const float threshold,
                             const int top_k,
                             std::vector<std::pair<float, int>> &score_index_vec)
{
    for (size_t i = 0; i < scores.size(); ++i) {
        if (scores[i] > threshold) {
            score_index_vec.push_back(std::make_pair(scores[i], i));
        }
    }
    std::stable_sort(score_index_vec.begin(), score_index_vec.end(), SortScorePairDescend<int>);
    if (top_k > 0 && top_k < (int) score_index_vec.size()) {
        score_index_vec.resize(top_k);
    }
}

template<typename BoxType>
inline void NMSFast_(const std::vector<BoxType> &bboxes,
                     const std::vector<float> &scores,
                     const float score_threshold,
                     const float nms_threshold,
                     const float eta,
                     const int top_k,
                     std::vector<int> &indices,
                     float (*computeOverlap)(const BoxType &, const BoxType &))
{
    CV_Assert(bboxes.size() == scores.size());
    std::vector<std::pair<float, int>> score_index_vec;
    GetMaxScoreIndex(scores, score_threshold, top_k, score_index_vec);

    // Do nms.
    float adaptive_threshold = nms_threshold;
    indices.clear();
    for (size_t i = 0; i < score_index_vec.size(); ++i) {
        const int idx = score_index_vec[i].second;
        bool keep = true;
        for (int k = 0; k < (int) indices.size() && keep; ++k) {
            const int kept_idx = indices[k];
            float overlap = computeOverlap(bboxes[idx], bboxes[kept_idx]);
            keep = overlap <= adaptive_threshold;
        }
        if (keep)
            indices.push_back(idx);
        if (keep && eta < 1 && adaptive_threshold > 0.5) {
            adaptive_threshold *= eta;
        }
    }
}

// copied from opencv 3.4, not exist in 3.0
template<typename _Tp>
static inline double jaccardDistance__(const cv::Rect_<_Tp> &a, const cv::Rect_<_Tp> &b)
{
    _Tp Aa = a.area();
    _Tp Ab = b.area();

    if ((Aa + Ab) <= std::numeric_limits<_Tp>::epsilon()) {
        // jaccard_index = 1 -> distance = 0
        return 0.0;
    }

    double Aab = (a & b).area();
    // distance = 1 - jaccard_index
    return 1.0 - Aab / (Aa + Ab - Aab);
}

template<typename T>
static inline float rectOverlap(const T &a, const T &b)
{
    return 1.f - static_cast<float>(jaccardDistance__(a, b));
}

void NMSBoxes(const std::vector<cv::Rect> &bboxes,
              const std::vector<float> &scores,
              const float score_threshold,
              const float nms_threshold,
              std::vector<int> &indices,
              const float eta = 1,
              const int top_k = 0)
{
    NMSFast_(bboxes, scores, score_threshold, nms_threshold, eta, top_k, indices, rectOverlap);
}

} // namespace cv_dnn

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

std::shared_ptr<NodeData> CvShapeBaseDetectorModel::outData(PortIndex portIndex)
{
    if (portIndex == 0) {
        return std::make_shared<ImageData>(_mat);
    } else if (portIndex == 1) {
        return std::make_shared<ResultData>(_res);
    } else {
        return nullptr;
    }
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
    Q_EMIT dataUpdated(1);
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
    _res.clear();

    cv::Mat test_img = d->mat();
    if (test_img.channels() == 1) {
        // gray -> rgb
        cv::cvtColor(test_img, test_img, cv::COLOR_GRAY2RGB);
    } else if (test_img.channels() == 4) {
        // 4 channel -> 3
        cv::cvtColor(test_img, test_img, cv::COLOR_RGBA2RGB);
    }

    bool is_find = false;

    line2Dup::Detector detector({4, 8}); // T 必须偶数

    std::vector<std::string> ids;
    ids.push_back("template");

    auto template_path = path_show->text().toStdString() + "/template.yaml";
    detector.readClasses(ids, template_path);

    // angle & scale are saved here, fetched by match id
    auto info_path = path_show->text().toStdString() + "/info.yaml";
    auto infos = shape_based_matching::shapeInfo_producer::load_infos(info_path);

    auto padding_path = path_show->text().toStdString() + "/padding.yaml";
    int padding = 0;
    cv::FileStorage fs(padding_path, cv::FileStorage::READ);
    padding = fs["padding"];
    fs.release();

    auto tmp_img = cv::imread(path_show->text().toStdString() + "/template.png");
    auto tmp_h = tmp_img.rows;
    auto tmp_w = tmp_img.cols;

    int stride = 16;
    int n = test_img.rows / stride;
    int m = test_img.cols / stride;
    cv::Rect roi(0, 0, stride * m, stride * n);
    test_img = test_img(roi).clone();

    auto matches = detector.match(test_img, threshold_edit->text().toFloat(), ids);

    vector<cv::Rect> boxes;
    vector<float> scores;
    vector<int> idxs;
    for (auto match : matches) {
        cv::Rect box;
        box.x = match.x;
        box.y = match.y;

        auto templ = detector.getTemplates("template", match.template_id);

        box.width = templ[0].width;
        box.height = templ[0].height;
        boxes.push_back(box);
        scores.push_back(match.similarity);
    }

    cv_dnn::NMSBoxes(boxes, scores, threshold_edit->text().toFloat(), 0.1f, idxs);

    if (!idxs.empty()) {
        is_find = true;
    }
    int id_c = 0;
    for (auto idx : idxs) {
        id_c += 1;
        auto match = matches[idx];
        auto templ = detector.getTemplates("template", match.template_id);

        float r_scaled_w = tmp_w / 2.0f * infos[match.template_id].scale;
        float r_scaled_h = tmp_h / 2.0f * infos[match.template_id].scale;
        // scaling won't affect this, because it has been determined by warpAffine
        // cv::warpAffine(src, dst, rot_mat, src.size()); last param
        float train_img_half_width = tmp_w / 2.0f + padding;
        float train_img_half_height = tmp_h / 2.0f + padding;

        // center x,y of train_img in test img
        float x = match.x - templ[0].tl_x + train_img_half_width;
        float y = match.y - templ[0].tl_y + train_img_half_height;

        int tw = templ[0].width / 2;
        int th = templ[0].height / 2;

        cv::Vec3b randColor;
        randColor[0] = rand() % 155 + 100;
        randColor[1] = rand() % 155 + 100;
        randColor[2] = rand() % 155 + 100;

        // 绘制特征点
        for (int i = 0; i < templ[0].features.size(); i++) {
            auto feat = templ[0].features[i];
            cv::circle(test_img, {feat.x + match.x, feat.y + match.y}, 3, randColor, -1);
        }

        // 绘制方框
        cv::RotatedRect rotatedRectangle({x, y},
                                         {2 * r_scaled_w, 2 * r_scaled_h},
                                         -infos[match.template_id].angle);

        cv::Point2f vertices[4];
        rotatedRectangle.points(vertices);
        for (int i = 0; i < 4; i++) {
            int next = (i + 1 == 4) ? 0 : (i + 1);
            cv::line(test_img, vertices[i], vertices[next], randColor, 3);
        }

        // 坐标轴
        float x_axis_l = tmp_w * infos[match.template_id].scale * 1.2;
        float y_axis_l = tmp_h * infos[match.template_id].scale * 1.2;
        cv::RotatedRect rotatedRectangleX({x, y}, {x_axis_l, 0}, -infos[match.template_id].angle);
        cv::Point2f verticesX[4];
        rotatedRectangleX.points(verticesX);
        cv::line(test_img, cv::Point(x, y), verticesX[3], cv::Scalar(0, 0, 255), 5);

        cv::RotatedRect rotatedRectangleY({x, y}, {0, y_axis_l}, -infos[match.template_id].angle);
        cv::Point2f verticesY[4];
        rotatedRectangleY.points(verticesY);
        cv::line(test_img, cv::Point(x, y), verticesY[2], cv::Scalar(0, 255, 0), 5);

        cv::circle(test_img, cv::Point(x, y), 5, cv::Scalar(255, 0, 0), -1);

        cv::putText(test_img,
                    (to_string(id_c)),
                    cv::Point(x, y),
                    cv::FONT_HERSHEY_SCRIPT_SIMPLEX,
                    4,
                    randColor,
                    2);

        // 输出: 角度&置信率.
        _res += ("id: " + std::to_string(id_c) + "\n"
                 + "similarity: " + std::to_string(match.similarity) + "\n"
                 + "angle: " + std::to_string(-infos[match.template_id].angle) + "\n"
                 + "scale: " + std::to_string(infos[match.template_id].scale) + "\n"
                 + "------------------------------" + "\n");
    }
    _mat = test_img;
    if (is_find) {
        _res = "flag: true\n------------------------------\nnum: " + std::to_string(id_c)
               + "\n------------------------------\n" + _res;
    } else {
        _res = "flag: false\n------------------------------\nnum: " + std::to_string(id_c)
               + "\n------------------------------\n";
    }

    int w = _label->width();
    int h = _label->height();

    auto pix = QPixmap::fromImage(cvMat2QImage(_mat));
    _label->setPixmap(pix.scaled(w, h, Qt::KeepAspectRatio));

    Q_EMIT dataUpdated(0);
    Q_EMIT dataUpdated(1);
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