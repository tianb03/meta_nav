#include <QWidget>
#include <QPushButton>

class MainWindow : public QWidget {

	Q_OBJECT

public:
	explicit MainWindow(QWidget *parent = NULL);
	~MainWindow() { }

private:

	void _init();

private slots:

	void _btn1_clicked();
	void _btn2_clicked();
	void _btn3_clicked();
	void _btn4_clicked();

private:

	QPushButton *_btn1;
	QPushButton *_btn2;
	QPushButton *_btn3;
	QPushButton *_btn4;
};
