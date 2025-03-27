#include "Application.h"

#include <QSerialPort>
#include <QString>

// Include the CSeatrac class
#include "seatrac_beacon.hpp"

/*!
 * \brief Application::Application
 * \param parent
 */
Application::Application(QObject *parent) : QObject(parent)
{
	port = nullptr;
	seatrac = nullptr;
}

/*!
 * \brief Application::connectToSeaTrac
 * \return
 */
bool Application::connectToSeaTrac()
{
	if (port)
		return false;

	port = new QSerialPort;
	connect(port, &QSerialPort::readyRead, this, &Application::readSeaTracPort);
	// TODO Change the port name to suit the end application
	port->setPortName("COM11");
	port->setBaudRate(115200);
	port->open(QIODevice::ReadWrite);

	seatrac = new CSeatrac;
	seatrac->OnCmdEncode.AttachMethod(this, &Application::writeSeaTracPort, nullptr);
	seatrac->OnCmdDecodeMsg.AttachMethod(this, &Application::messageDecoded, nullptr);
	seatrac->OnCmdDecodeLine.AttachMethod(this, &Application::lineDecoded, nullptr);

	// SYS messages
	seatrac->OnSysAlive.AttachMethod(this, &Application::seatracSysAlive, nullptr);
	seatrac->OnSysInfo.AttachMethod(this, &Application::seatracSysInfo, nullptr);
	seatrac->OnStatus.AttachMethod(this, &Application::seatracStatus, nullptr);

	// PING response message
	seatrac->OnPingResponse.AttachMethod(this, &Application::seatracPingResponse, nullptr);

	return true;
}

/*!
 * \brief Application::disconnectFromSeaTrac
 */
void Application::disconnectFromSeaTrac()
{
	if (!port)
		return;

	disconnect(port, &QSerialPort::readyRead, this, &Application::readSeaTracPort);

	port->close();
	delete port;
	port = nullptr;
}



void Application::writeSeaTracPort(pointer context, PSeatracCmdEncodeParams params)
{
	Q_UNUSED(context)

	QByteArray buffer((char *)params->Buffer, params->Length);
	// Write the string to the underlying serial port
	port->write(buffer);
}

void Application::readSeaTracPort()
{
	// Read all available data
	QByteArray data = port->readAll();
	// Convert the data to a QString
	QString sentence(data);

	std::string str = sentence.toStdString();

	// Decode the string
	seatrac->CmdDecode(str);
}

void Application::messageDecoded(pointer context, PSeatracCmdDecodeMsgParams params)
{
	Q_UNUSED(context)
	Q_UNUSED(params)

	/*
	static quint32 lines = 1;

	// Update the journal
	QString message = QString::fromLatin1("#%1\t%2\t%3")
			.arg(lines)
			.arg(QDateTime::currentDateTime().toString())
			.arg("Message");

	ui->journal->insertPlainText(message);

	lines++;
	*/
}

void Application::lineDecoded(pointer context, PSeatracCmdDecodeLineParams params)
{
	Q_UNUSED(context)
	Q_UNUSED(params)

	/*
	QString str(params->Buffer);
	qDebug() << str;
	*/
}


void Application::seatracSysAlive(pointer context, PSeatracSysAliveParams params)
{
	Q_UNUSED(context)
	Q_UNUSED(params)
}

void Application::seatracSysInfo(pointer context, PSeatracSysInfoParams params)
{
	Q_UNUSED(context)

	QString hardware = QString::fromLatin1("%1")
			.arg(QString::number(params->Hardware.SerialNumber));

	QString bootloader = QString::fromLatin1("v%1.%2.%3")
			.arg(QString::number(params->BootApp.VersionMajor))
			.arg(QString::number(params->BootApp.VersionMinor))
			.arg(QString::number(params->BootApp.VersionBuild));

	QString application = QString::fromLatin1("v%1.%2.%3")
			.arg(QString::number(params->MainApp.VersionMajor))
			.arg(QString::number(params->MainApp.VersionMinor))
			.arg(QString::number(params->MainApp.VersionBuild));

	double_t hours = params->Seconds / 3600.0F;
	double_t minutes = (hours - (int)hours) * 60.0;
	double_t seconds = (minutes - (int)minutes) * 60.0;

	QString runtime = QString::fromLatin1("%1:%2:%3")
			.arg(QString::number((int)hours))
			.arg(QString::number((int)minutes))
			.arg(QString::number((int)seconds));

	//emit onSeatracSysInfo(hardware, bootloader, application, runtime);

	/*
	// Update the system info widgets
	ui->hardwareInfo->setText(hardware);
	ui->bootloaderInfo->setText(bootloader);
	ui->applicationInfo->setText(application);
	ui->runTime->setText(runtime);*/
}

void Application::seatracStatus(pointer context, PSeatracStatusParams params)
{
	Q_UNUSED(context)

	// Get the flags
	quint8 flags = params->Status.Flags;

	/*
	emit onSeatracStatus(params->Status.EnvSupply, params->Status.EnvTemp,
						 params->Status.EnvPressure, params->Status.EnvDepth,
						 params->Status.EnvVos);

	emit onSeatracEulerAngles(params->Status.AttitudeYaw, params->Status.AttitudePitch,
							  params->Status.AttitudeRoll);
*/
	/*
	if (IS_BIT_SET(flags, ST_STATUS_FLAGS_BNO055_STATUS_BIT)) {
		emit onSeatracAhrsStatus(params->Status.SysFlag, params->Status.AccFlag,
								 params->Status.MagFlag, params->Status.GyroFlag);
	}

	if (IS_BIT_SET(flags, ST_STATUS_FLAGS_BNO055_QUAT_BIT)) {
		emit onSeatracQuaternions(params->Status.Q0, params->Status.Q1,
								  params->Status.Q2, params->Status.Q3);
	}
	*/
}


void Application::seatracPingResponse(pointer context, PSeatracPingResponseParams params)
{
	(void)(context);

	//TODO: On the X010's response, we parse the message
	/*
	if (params->CmdStatus == ST_CST_OK) {
		this->beep();
	}
	else {
		this->boop();
	}*/

	emit pingResponse(params->Response);
}

void Application::sendPing(ESeatracBeaconId id)
{
	seatrac->PingSend(id, ST_AMSG_REQU);
}
