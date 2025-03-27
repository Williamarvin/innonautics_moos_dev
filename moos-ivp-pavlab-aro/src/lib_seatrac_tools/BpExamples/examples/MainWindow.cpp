#include "MainWindow.h"
#include "ui_MainWindow.h"

#include "Application.h"

Application App;

MainWindow::MainWindow(QWidget *parent)
	: QMainWindow(parent)
	, ui(new Ui::MainWindow)
{
	ui->setupUi(this);

	connect(ui->connect, &QPushButton::clicked, &App, &Application::connectToSeaTrac);
	connect(ui->ping, &QPushButton::clicked, this, &MainWindow::sendPing);

	// Connect to the ping response signal
	connect(&App, &Application::pingResponse, this, &MainWindow::handlePingResponse);
}

MainWindow::~MainWindow()
{
	delete ui;
}

void MainWindow::sendPing()
{
	uint32_t id = ui->id->value();
	QString status = QString::fromLatin1("Pinging %1...")
			.arg(id);

	ui->status->setText(status);

	ESeatracBeaconId dest = (ESeatracBeaconId)id;
	App.sendPing(dest);
}

void MainWindow::handlePingResponse(TSeatracAcoFix fix)
{
	QString fixInfo = QString::fromLatin1("Range: %1m, N: %2m, E: %3m")
			.arg(fix.Info.RangeDist)
			.arg(fix.Info.PositionNorthing)
			.arg(fix.Info.PositionEasting);

	ui->status->setText(fixInfo);
}

