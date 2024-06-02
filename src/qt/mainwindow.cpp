#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <random>
double qt_time = 0;
double qt_time_dt =0.001;

typedef enum { JOINT_POS, JOINT_VEL, JOINT_TORQ ,EXT_WRENCH,FLAG_V,FLAG_LAMBDA} ENUM_DISPLAY_FLAG;
int display_flag = FLAG_LAMBDA;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    GraphInit();
}

MainWindow::~MainWindow()
{
    delete ui;
}
void MainWindow::GraphInit()
{

    // include this section to fully disable antialiasing for higher performance:
    /*
    customPlot->setNotAntialiasedElements(QCP::aeAll);
    QFont font;
    font.setStyleStrategy(QFont::NoAntialias);
    customPlot->xAxis->setTickLabelFont(font);
    customPlot->yAxis->setTickLabelFont(font);
    customPlot->legend->setFont(font);
    */
    QVector<QColor> colors;
    colors.append(QColor(255, 0, 0));      // 빨간색
    colors.append(QColor(0, 255, 0));      // 녹색
    colors.append(QColor(0, 0, 255));      // 파란색
    colors.append(QColor(255, 165, 0));    // 오렌지색
    colors.append(QColor(128, 0, 128));    // 자주색
    colors.append(QColor(0, 128, 128));    // 청록색
    colors.append(QColor(255, 192, 203));  // 핑크색

    for(int i =0;i<JOINTNUM;i++){
        ui->widget_RealTimeGraph1->addGraph();
        ui->widget_RealTimeGraph1->graph(i)->setPen(QPen(colors.at(i)));

        ui->widget_RealTimeGraph2->addGraph();
        ui->widget_RealTimeGraph2->graph(i)->setPen(QPen(colors.at(i)));

        ui->widget_RealTimeGraph3->addGraph();
        ui->widget_RealTimeGraph3->graph(i)->setPen(QPen(colors.at(i)));

    }
    
 connect(ui->pushButton_lambda, SIGNAL(clicked()), this, SLOT(on_pushButton_lambda_clicked()));

    QSharedPointer<QCPAxisTickerTime> timeTicker(new QCPAxisTickerTime);
    timeTicker->setTimeFormat("%h:%m:%s");
    ui->widget_RealTimeGraph1->xAxis->setTicker(timeTicker);
    ui->widget_RealTimeGraph1->axisRect()->setupFullAxesBox();
    ui->widget_RealTimeGraph1->yAxis->setRange(-3.141592*2.0, 3.141592*2.0);
    ui->widget_RealTimeGraph1->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
    ui->widget_RealTimeGraph1->legend->setVisible(true);
    ui->widget_RealTimeGraph1->legend->setBrush(QBrush(QColor(255,255,255,150)));
    ui->widget_RealTimeGraph1->axisRect()->insetLayout()->setInsetAlignment(0,Qt::AlignLeft|Qt::AlignTop);
    // make left and bottom axes transfer their ranges to right and top axes:
    connect(ui->widget_RealTimeGraph1->xAxis, SIGNAL(rangeChanged(QCPRange)), ui->widget_RealTimeGraph1->xAxis2, SLOT(setRange(QCPRange)));
    connect(ui->widget_RealTimeGraph1->yAxis, SIGNAL(rangeChanged(QCPRange)), ui->widget_RealTimeGraph1->yAxis2, SLOT(setRange(QCPRange)));

    ui->widget_RealTimeGraph2->xAxis->setTicker(timeTicker);
    ui->widget_RealTimeGraph2->axisRect()->setupFullAxesBox();
    ui->widget_RealTimeGraph2->yAxis->setRange(-3.141592*2.0, 3.141592*2.0);
    ui->widget_RealTimeGraph2->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
    ui->widget_RealTimeGraph2->legend->setVisible(true);
    ui->widget_RealTimeGraph2->legend->setBrush(QBrush(QColor(255,255,255,150)));
    ui->widget_RealTimeGraph2->axisRect()->insetLayout()->setInsetAlignment(0,Qt::AlignLeft|Qt::AlignTop);
    // make left and bottom axes transfer their ranges to right and top axes:
    connect(ui->widget_RealTimeGraph2->xAxis, SIGNAL(rangeChanged(QCPRange)), ui->widget_RealTimeGraph2->xAxis2, SLOT(setRange(QCPRange)));
    connect(ui->widget_RealTimeGraph2->yAxis, SIGNAL(rangeChanged(QCPRange)), ui->widget_RealTimeGraph2->yAxis2, SLOT(setRange(QCPRange)));


    ui->widget_RealTimeGraph3->xAxis->setTicker(timeTicker);
    ui->widget_RealTimeGraph3->axisRect()->setupFullAxesBox();
    ui->widget_RealTimeGraph3->yAxis->setRange(-3.141592*2.0, 3.141592*2.0);
    ui->widget_RealTimeGraph3->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
    ui->widget_RealTimeGraph3->legend->setVisible(true);
    ui->widget_RealTimeGraph3->legend->setBrush(QBrush(QColor(255,255,255,150)));
    ui->widget_RealTimeGraph3->axisRect()->insetLayout()->setInsetAlignment(0,Qt::AlignLeft|Qt::AlignTop);
    // make left and bottom axes transfer their ranges to right and top axes:
    connect(ui->widget_RealTimeGraph3->xAxis, SIGNAL(rangeChanged(QCPRange)), ui->widget_RealTimeGraph3->xAxis2, SLOT(setRange(QCPRange)));
    connect(ui->widget_RealTimeGraph3->yAxis, SIGNAL(rangeChanged(QCPRange)), ui->widget_RealTimeGraph3->yAxis2, SLOT(setRange(QCPRange)));



    // setup a timer that repeatedly calls MainWindow::realtimeDataSlot:
    connect(&DataTimer, SIGNAL(timeout()), this, SLOT(RealtimeDataSlot()));
    connect(&PhysicsTimer, SIGNAL(timeout()), this, SLOT(RealtimePhysicsSlot()));
    //DataTimer.setTimerType(Qt::PreciseTimer);
    DataTimer.start(10); // Interval 0 means to refresh as fast as possible
    

}
void MainWindow::RealtimePhysicsSlot(){
    qt_time+=qt_time_dt;
}
void MainWindow::RealtimeDataSlot()
{
    static QTime time(QTime::currentTime());
    qsrand(QTime::currentTime().msecsSinceStartOfDay());

      // calculate two new data points:
      double key = time.elapsed()/1000.0; // time elapsed since start of demo, in seconds
      static double lastPointKey = 0;
    	//std::lock_guard<std::mutex> lock(g_pages_mutex); 
        HJVec q1 = HJVec::Zero();
        HJVec q2 = HJVec::Zero();
        HJVec q3 = HJVec::Zero();
        for(int j =0;j<4;j++){
            switch(display_flag){
                case FLAG_V: 
                q1 = info.act.q_list.at(0);
                q2 = info.act.q_list.at(1);
                q3 = info.act.q_list.at(2);
                 ui->widget_RealTimeGraph1->graph(j)->addData(key, q1(j));        
                ui->widget_RealTimeGraph2->graph(j)->addData(key, q2(j));
                ui->widget_RealTimeGraph3->graph(j)->addData(key, q3(j));
                
                break;
                case FLAG_LAMBDA: 
                
                ui->widget_RealTimeGraph1->graph(j)->addData(key, info.des.V[j]);
                ui->widget_RealTimeGraph2->graph(j)->addData(key, info.des.Vdot[j]);
                ui->widget_RealTimeGraph3->graph(j)->addData(key, info.des.Vddot[j]);
                
                break;
            }
        }
            
      ui->widget_RealTimeGraph1->xAxis->setRange(key,5,Qt::AlignRight);
      ui->widget_RealTimeGraph1->replot();

       ui->widget_RealTimeGraph2->xAxis->setRange(key,5,Qt::AlignRight);
      ui->widget_RealTimeGraph2->replot();


        ui->widget_RealTimeGraph3->xAxis->setRange(key,5,Qt::AlignRight);
      ui->widget_RealTimeGraph3->replot();
      // calculate frames per second:
      static double lastFpsKey;
      static int frameCount;
      ++frameCount;
      if (key-lastFpsKey > 1) // average fps over 2 seconds
      {
        ui->statusBar->showMessage(
              QString("%1 FPS, Total Data points: %2")
              .arg(frameCount/(key-lastFpsKey), 0, 'f', 0)
              .arg(ui->widget_RealTimeGraph1->graph(0)->data()->size()+ui->widget_RealTimeGraph1->graph(1)->data()->size())
              , 0);
        lastFpsKey = key;
        frameCount = 0;
      }
     
}

void MainWindow::on_pushButton_V_clicked()
{
    display_flag = FLAG_V;
    double MAX_POS = M_PI;
    ui->widget_RealTimeGraph1->yAxis->setRange(-MAX_POS, MAX_POS);
    ui->widget_RealTimeGraph2->yAxis->setRange(-MAX_POS, MAX_POS);
    ui->widget_RealTimeGraph3->yAxis->setRange(-MAX_POS, MAX_POS);
    std::cout<<"FLAG_V "<<std::endl;
    for(int i = 0; i < 6; i++) {
        ui->widget_RealTimeGraph1->graph(i)->setName(QString("V%1").arg(i));
        ui->widget_RealTimeGraph2->graph(i)->setName(QString("Vdot%1").arg(i));
        ui->widget_RealTimeGraph3->graph(i)->setName(QString("Vddot%1").arg(i));
    }

    ui->widget_RealTimeGraph1->replot();
    ui->widget_RealTimeGraph2->replot();
    ui->widget_RealTimeGraph3->replot();

    std::cout << "FLAG_V" << std::endl;    

}

void MainWindow::on_pushButton_lambda_clicked()
{
    display_flag = FLAG_LAMBDA;
    double MAX_POS = M_PI;
    ui->widget_RealTimeGraph1->yAxis->setRange(-MAX_POS, MAX_POS);
    ui->widget_RealTimeGraph2->yAxis->setRange(-MAX_POS, MAX_POS);
    ui->widget_RealTimeGraph3->yAxis->setRange(-MAX_POS, MAX_POS);
    std::cout<<"FLAG_LAMBDA "<<std::endl;
        for(int i = 0; i < 6; i++) {
        ui->widget_RealTimeGraph1->graph(i)->setName(QString("V_des%1").arg(i));
        ui->widget_RealTimeGraph2->graph(i)->setName(QString("Vdot_des%1").arg(i));
        ui->widget_RealTimeGraph3->graph(i)->setName(QString("Vddot_des%1").arg(i));
    }

    ui->widget_RealTimeGraph1->replot();
    ui->widget_RealTimeGraph2->replot();
    ui->widget_RealTimeGraph3->replot();

    std::cout << "FLAG_LAMBDA" << std::endl;
}


