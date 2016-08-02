#include <QApplication>
#include "mainwindow.h"

int main(int argc, char *argv[])
{
	QApplication app(argc, argv);
	MainWindow *wid = new MainWindow;
	wid->show();

	return app.exec();
}
