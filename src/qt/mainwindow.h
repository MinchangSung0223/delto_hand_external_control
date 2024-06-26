#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QtWidgets>
#include "../main.h"
namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
//  void physics_callback2(union sigval sv);
private slots:
    void RealtimeDataSlot();

    void RealtimePhysicsSlot();
    void on_pushButton_V_clicked();

    void on_pushButton_lambda_clicked();


private:
    Ui::MainWindow *ui;
    QTimer DataTimer;
    QTimer PhysicsTimer;

    void GraphInit();

};

#endif // MAINWINDOW_H