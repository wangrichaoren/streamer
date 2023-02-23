//
// Created by wrc on 23-2-10.
//

#include "Widget/StreamerMainWindow.hpp"
#include <QApplication>

StreamerMainWindow *smw;

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);

    StreamerMainWindow streamer_main_window(nullptr);
    streamer_main_window.show();
    smw = &streamer_main_window;

    return QApplication::exec();

}
