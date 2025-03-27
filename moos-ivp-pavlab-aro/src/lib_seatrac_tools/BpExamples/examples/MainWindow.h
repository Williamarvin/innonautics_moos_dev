#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

#include "types.h"
#include "seatrac_cmdproc.hpp"

class MainWindow : public QMainWindow
{
	Q_OBJECT

public:
	MainWindow(QWidget *parent = nullptr);
	~MainWindow();

private:
	Ui::MainWindow *ui;

	void sendPing();

private slots:
	void handlePingResponse(TSeatracAcoFix fix);
};
#endif // MAINWINDOW_H
