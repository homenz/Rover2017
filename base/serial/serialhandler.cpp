/**
 *The SerialHandler class will handle serial interfacing with the radio that
 * will transmit data to the rover. This class will contain methods for sending
 * packets to the rover. It will also emit read packets from the serial and
 * emit signals when a packet is recieved.
 **/


#include "serial/serialhandler.h"
#include "serial/singleton.h"
#include <QSerialPortInfo>
#include <QBuffer>


SerialHandler* SerialHandler::createInstance()
{
    return new SerialHandler();
}

SerialHandler* SerialHandler::instance()
{
    return Singleton<SerialHandler>::instance(SerialHandler::createInstance);
}

SerialHandler::~SerialHandler()
{
    delete m_packets;
}


void SerialHandler::run()
{
    qDebug() << "starting serial read";
    eventLoop();
}

void SerialHandler::eventLoop()
{
    while (m_run){
        if(m_packets->device()->bytesAvailable() >= 2) {
            qDebug() << "read some bytes";
            quint8 start, size;
            *m_packets->datastream() >> start;

            qDebug() << start;
            if(start != 0x01) continue;
            qDebug() << "sucessfully read start byte";
            *m_packets->datastream() >> size;

            if(size < 3) continue;
            qDebug() << "read size of: " << size;
            qDebug() << "about to parse packet";
            while(m_packets->device()->bytesAvailable() < size) {
                msleep(50);
            }
            m_packets->parsePacket(size);
            /* readData.append(m_packets->device()->readAll()); */
            //qDebug() << readData.toHex();
        }

    }
    qDebug() << "exciting serial read";
}

void SerialHandler::stop() {
    m_run = false;
}


void SerialHandler::connectDevice()
{
    QList<QSerialPortInfo> serialPorts = QSerialPortInfo::availablePorts();
    for(int i = 0; i < serialPorts.size(); i++) {
        // for now
        qDebug() << serialPorts[i].portName();
        if(serialPorts[i].portName() == "ttyUSB0") {
            qDebug() << "identifyed serial";
            QSerialPort *serial = new QSerialPort(serialPorts[i]);
            if(!serial->open(QIODevice::ReadWrite)) {
                qDebug() << tr("error %1").arg(serial->error());
            }
            serial->setBaudRate(9600);
            serial->setDataBits(QSerialPort::Data8);
            serial->setParity(QSerialPort::NoParity);
            serial->setStopBits(QSerialPort::OneStop);

            setDevice(serial);
        }
    }
}

void SerialHandler::setDevice(QIODevice *d)
{
    m_packets->setDevice(d);
}

void SerialHandler::setBuffer(QByteArray *a)
{
    QBuffer *buffer = new QBuffer(a);
    m_packets->setDevice(buffer);
}

QIODevice *SerialHandler::device()
{
    return m_packets->device();
}

SerialHandler::SerialHandler(QObject *parent)
    : QThread(parent),
      m_packets(new Packets())
{
    m_run = true;
}
