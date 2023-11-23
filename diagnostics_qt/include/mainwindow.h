#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QProcess>
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include "rclcomm.h"
#include <iostream>
QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private:
    Ui::MainWindow *ui;
    rclcomm *commNode;
public slots:
    void updateDiagnostics(QString);
    void updateTreeWidget(QString,QString,QString);
    void clearTreeWidget();
};
#endif // MAINWINDOW_H
