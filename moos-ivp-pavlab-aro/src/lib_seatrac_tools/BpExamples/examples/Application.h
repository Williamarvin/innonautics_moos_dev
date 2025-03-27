#ifndef APPLICATION_H
#define APPLICATION_H

#include <QObject>

#include "types.h"
#include "seatrac_cmdproc.hpp"

class QSerialPort;
class CSeatrac;

class Application : public QObject
{
	Q_OBJECT
public:
	explicit Application(QObject *parent = nullptr);

	bool connectToSeaTrac();
	void disconnectFromSeaTrac();

	void sendPing(ESeatracBeaconId id);

private:
	QSerialPort *port;
	CSeatrac *seatrac;

	void writeSeaTracPort(pointer context, PSeatracCmdEncodeParams params);
	void readSeaTracPort();

	void messageDecoded(pointer context, PSeatracCmdDecodeMsgParams params);
	void lineDecoded(pointer context, PSeatracCmdDecodeLineParams params);

	void seatracSysAlive(pointer context, PSeatracSysAliveParams params);
	void seatracSysInfo(pointer context, PSeatracSysInfoParams params);
	//void seatracSysEngineering(pointer context, PSeatracSysEngineeringParams params);
	void seatracStatus(pointer context, PSeatracStatusParams params);

	void seatracPingResponse(pointer context, PSeatracPingResponseParams params);

signals:
	void pingResponse(TSeatracAcoFix fix);

};

extern Application App;

#endif // APPLICATION_H
