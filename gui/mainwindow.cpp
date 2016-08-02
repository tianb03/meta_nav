#include "mainwindow.h"
#include <QHBoxLayout>
#include <QLabel>

MainWindow::MainWindow(QWidget *parent)
	: QWidget(parent)
	, _btn1(new QPushButton(tr("to eat")))
	, _btn2(new QPushButton(tr("to sleep")))
	, _btn3(new QPushButton(tr("to fuck")))
	, _btn4(new QPushButton(tr("to suck")))
{
	_init();
	connect(_btn1, SIGNAL(clicked()), this, SLOT(_btn1_clicked()));
	connect(_btn2, SIGNAL(clicked()), this, SLOT(_btn2_clicked()));
	connect(_btn3, SIGNAL(clicked()), this, SLOT(_btn3_clicked()));
	connect(_btn4, SIGNAL(clicked()), this, SLOT(_btn4_clicked()));
	setFixedSize(sizeHint());
}

void MainWindow::_init()
{
	QHBoxLayout *lay = new QHBoxLayout;
	lay->addWidget(_btn1);
	lay->addWidget(_btn2);
	lay->addWidget(_btn3);
	lay->addWidget(_btn4);
	setLayout(lay);
}

void MainWindow::_btn1_clicked()
{
	QLabel *lb = new QLabel("to eat clicked");
	lb->setFixedSize(200, 100);
	1lb->show();
}

void MainWindow::_btn2_clicked()
{
	QLabel *lb = new QLabel("to sleep clicked");
	lb->setFixedSize(200, 100);
	lb->show();
}

void MainWindow::_btn3_clicked()
{
	QLabel *lb = new QLabel("to fuck clicked");
	lb->setFixedSize(200, 100);
	lb->show();
}

void MainWindow::_btn4_clicked()
{
	QLabel *lb = new QLabel("to suck clicked");
	lb->setFixedSize(200, 100);
	lb->show();
}
