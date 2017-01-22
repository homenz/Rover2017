#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QVector2D>
#include <QVector>
#include <QMainWindow>
#include <QSerialPort>
#include <QDebug>
#include <QQuickWidget>
#include <QThread>
#include <QTime>

#include "serial/serialhandler.h"
#include "inputs/controllerhandler.h"
#include "threadarray.h"


namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    virtual void closeEvent (QCloseEvent *event);

    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private:
    Ui::MainWindow *ui;

    ThreadArray threadarray;

    QSerialPort output;

    bool _serialRunning;

    ControllerHandler *m_inputs;
    SerialHandler *m_serial;

public slots:

private slots:

    void on_actionStart_Thread_triggered();

    void on_actionStop_Thread_triggered();

    void on_actionStart_Thread_2_triggered();

    void on_actionStop_Thread_2_triggered();

    void on_actionPing_triggered();

    void on_actionAutodetect_Serial_triggered();

    void on_actionIdentify_controllers_triggered();

    void on_exit_clicked();

signals:

    void startSerial();
    void stopSerial();
    void startInputs();
    void stopInputs();

    void startReadIn();
    void stopReadIn();
    void closeThreads();
    void startThreads();


};

#endif // MAINWINDOW_H
