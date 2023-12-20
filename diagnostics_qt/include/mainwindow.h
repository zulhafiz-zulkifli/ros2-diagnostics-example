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
    QTreeWidgetItem *invisibleRoot;
public slots:
    void updateTreeWidget(QString,QString,QString);
    void traverseTree(QTreeWidgetItem *parentItem, QString name, QString message, QString level);
    void clearTreeWidget();
};
#endif // MAINWINDOW_H
