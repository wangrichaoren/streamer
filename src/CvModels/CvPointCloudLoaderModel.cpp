#include "CvModels/CvPointCloudLoaderModel.hpp"

#include <QtCore/QDir>
#include <QtCore/QEvent>

#include "Widget/StreamerMainWindow.hpp"
#include <QJsonArray>
#include <QtWidgets/QFileDialog>

CvPointCloudLoaderModel::CvPointCloudLoaderModel()
    : _box(new QGroupBox())
    , _layout(new QVBoxLayout(_box))
    , load_btn(new QPushButton("导入", _box))
{
    _box->setMinimumSize(200, 200);
    _box->setMaximumSize(500, 500);
    path_line = new QLineEdit();
    path_line->setPlaceholderText("path paste to here");
    _layout->addWidget(path_line);
    _layout->addWidget(load_btn);
    _box->setLayout(_layout);
    _box->resize(200, 200);

    _box->installEventFilter(this);

    connect(path_line, &QLineEdit::editingFinished, [=] {
        auto filePath = path_line->text().toStdString();
        if (filePath.empty()) {
            return;
        }
        if (!checkFileExists(filePath)) {
            return;
        }

        compute();
    });

    connect(load_btn, &QPushButton::clicked, [=] {
        QString fileName = QFileDialog::getOpenFileName(nullptr,
                                                        tr("Open PC"),
                                                        QDir::homePath(),
                                                        tr("Point Cloud Files (*.pcd *.ply)"));
        if (fileName.isEmpty()) {
            return false;
        }

        if (!checkFileExists(fileName.toStdString())) {
            return false;
        }

        path_line->setText(fileName);

        compute();
        return true;
    });
}

unsigned int CvPointCloudLoaderModel::nPorts(PortType portType) const
{
    unsigned int result = 1;

    switch (portType) {
    case PortType::In:
        result = 0;
        break;

    case PortType::Out:
        result = 1;

    default:
        break;
    }
    return result;
}

bool CvPointCloudLoaderModel::eventFilter(QObject *object, QEvent *event)
{
    if (object == _box) {
        if (_pc->empty()){
            return false;
        }
        if (event->type() == QEvent::MouseButtonPress) {
            extern StreamerMainWindow *smw;
            smw->updateVTK(_pc);
            return true;
        }
    }
    return false;
}

NodeDataType CvPointCloudLoaderModel::dataType(PortType const, PortIndex const) const
{
    return PointCloudData().type();
}

std::shared_ptr<NodeData> CvPointCloudLoaderModel::outData(PortIndex)
{
    return std::make_shared<PointCloudData>(_pc);
}
QJsonObject CvPointCloudLoaderModel::save() const
{
    auto saver = NodeDelegateModel::save();
    saver["path"] = path_line->text();
    return saver;
}
void CvPointCloudLoaderModel::load(const QJsonObject &js)
{
    path_line->setText(js["path"].toString());
    path_line->editingFinished();
}

void CvPointCloudLoaderModel::compute()
{
    // check .pcd or .ply
    auto fileName=path_line->text().toStdString();
    std::string suffixStr = fileName.substr(fileName.find_last_of('.') + 1);//获取文件后缀
    if (suffixStr == "pcd") {
           pcl::io::loadPCDFile(path_line->text().toStdString(), *_pc);
       } else if (suffixStr == "ply") {
           pcl::io::loadPLYFile(path_line->text().toStdString(), *_pc);
       }

    extern StreamerMainWindow *smw;
    smw->updateVTK(_pc);

    Q_EMIT dataUpdated(0);
}
