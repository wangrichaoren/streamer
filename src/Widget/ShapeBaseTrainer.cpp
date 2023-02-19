//
// Created by wrc on 23-2-18.
//

// You may need to build the project (run Qt uic code generator) to get "ui_ShapeBaseTrainer.h" resolved

#include "Widget/ShapeBaseTrainer.h"
#include "Widget/ui_ShapeBaseTrainer.h"

ShapeBaseTrainerDialog::ShapeBaseTrainerDialog(QWidget *parent)
    : QDialog(parent)
    , ui(new Ui::ShapeBaseTrainerDialog)
{
    ui->setupUi(this);

    ui->graphicsView->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    ui->graphicsView->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

    ui->graphicsView_2->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    ui->graphicsView_2->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

    m_view = new MyGraphicsView(ui->graphicsView, true);
    m_view2 = new MyGraphicsView(ui->graphicsView_2, false);

    connect(ui->open_pushButton, &QPushButton::clicked, [=] {
        QString fileName = QFileDialog::getOpenFileName(this,
                                                        tr("Open Image"),
                                                        QDir::homePath(),
                                                        tr("Image Files (*.png *.jpg *.bmp)"));
        if (fileName.isEmpty()) {
            return;
        }
        auto pix = QImage(fileName.toStdString().data());
        m_view->graphics(pix, fileName.toStdString());
    });

    connect(ui->exit_pushButton, &QPushButton::clicked, [=] { this->close(); });

    connect(ui->train_pushButton, &QPushButton::clicked, [=] {
        if (!m_view->hasRoiRect()) {
            createMessageBox(this,
                             ":icons/shy.png",
                             "未完成标注",
                             "请选择一张预标注图片并完成标注后再执行训练!\n[标注方法]:双击图片区域,"
                             "生成的方形框拖拽至目标,点击方形框边缘能调控大小.",
                             1,
                             {"确认"});
            return;
        }
        line2Dup::Detector detector(ui->num_lineEdit->text().toInt(), {4, 8});

        auto img = m_view->getRoiRectToMat();
        assert(!img.empty() && "get roi img error!");
        cv::Mat mask = cv::Mat(img.size(), CV_8UC1, {255});

        // 填充以避免旋转丢失特征
        int padding;
        // 求填充的大小，而不是固定值
        auto max_img_h = ui->scale2_lineEdit->text().toFloat() * img.rows;
        auto max_img_w = ui->scale2_lineEdit->text().toFloat() * img.cols;
        if (max_img_h > max_img_w) {
            padding = (max_img_h - max_img_w) / 2;
        } else {
            padding = (max_img_w - max_img_h) / 2;
        }
        //        std::cout<<"padding: "<<padding<<std::endl;

        cv::Mat padded_img = cv::Mat(img.rows + 2 * padding,
                                     img.cols + 2 * padding,
                                     img.type(),
                                     cv::Scalar::all(0));
        img.copyTo(padded_img(cv::Rect(padding, padding, img.cols, img.rows)));

        cv::Mat padded_mask = cv::Mat(mask.rows + 2 * padding,
                                      mask.cols + 2 * padding,
                                      mask.type(),
                                      cv::Scalar::all(0));
        mask.copyTo(padded_mask(cv::Rect(padding, padding, img.cols, img.rows)));

        shape_based_matching::shapeInfo_producer shapes(padded_img, padded_mask);
        shapes.angle_range = {ui->down_r_lineEdit->text().toFloat(),
                              ui->up_r_lineEdit->text().toFloat()};
        shapes.angle_step = ui->r_step_lineEdit->text().toFloat();
        shapes.scale_range = {ui->scale1_lineEdit->text().toFloat(),
                              ui->scale2_lineEdit->text().toFloat()};
        shapes.scale_step = ui->scale_step_lineEdit->text().toFloat();

        shapes.produce_infos();
        std::vector<shape_based_matching::shapeInfo_producer::Info> infos_have_templ;
        string class_id = "template";

        for (auto &info : shapes.infos) {
            cv::Mat to_show = shapes.src_of(info);

            //            std::cout << "\ninfo.angle: " << info.angle << std::endl;
            int templ_id = detector.addTemplate(shapes.src_of(info), class_id, shapes.mask_of(info));
            //            std::cout << "templ_id: " << templ_id << std::endl;

            auto templ = detector.getTemplates("template", templ_id);
            for (int i = 0; i < templ[0].features.size(); i++) {
                auto feat = templ[0].features[i];
                cv::circle(to_show,
                           {feat.x + templ[0].tl_x, feat.y + templ[0].tl_y},
                           3,
                           {0, 0, 255},
                           -1);
            }

            m_view2->graphics(to_show);
            cv::waitKey(0);

            //            std::cout << "templ_id: " << templ_id << std::endl;
            if (templ_id != -1) {
                infos_have_templ.push_back(info);
            }
        }

        // dir save
        QString dirName = QFileDialog::getExistingDirectory(this,
                                                            tr("Train Successful!"),
                                                            QDir::homePath());
        if (dirName.isEmpty()) {
            return;
        }

        detector.writeClasses(dirName.toStdString() + "/template.yaml");
        shapes.save_infos(infos_have_templ, dirName.toStdString() + "/info.yaml");
        cv::imwrite(dirName.toStdString() + "/template.png", img);
        // add padding file
        cv::FileStorage fs(dirName.toStdString() + "/padding.yaml", cv::FileStorage::WRITE);
        fs << "padding" << padding;
        fs.release();

        std::cout << "train end" << std::endl << std::endl;
        createMessageBox(this, ":icons/cool.png", "成功", "模板保存完成!", 1, {"返回"});

        m_view->clearView();
        m_view2->clearView();
    });

    connect(ui->downlineEdit, &QLineEdit::editingFinished, [=] {
        int val = ui->downlineEdit->text().toInt();
        if (val % 2 != 0) {
            ui->downlineEdit->setText(std::to_string(val + 1).data());
        }
    });

    connect(ui->uplineEdit, &QLineEdit::editingFinished, [=] {
        int val = ui->uplineEdit->text().toInt();
        if (val % 2 != 0) {
            ui->uplineEdit->setText(std::to_string(val + 1).data());
        }
    });

    connect(ui->num_lineEdit, &QLineEdit::editingFinished, [=] {
        int val = ui->num_lineEdit->text().toInt();
        if (val < 100) {
            ui->num_lineEdit->setText(std::to_string(100).data());
        }
    });
}

ShapeBaseTrainerDialog::~ShapeBaseTrainerDialog()
{
    delete m_view;
    delete m_view2;
    delete ui;
}
