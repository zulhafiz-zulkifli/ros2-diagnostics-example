#include "mainwindow.h"
#include "./ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    commNode=new rclcomm();

    connect(commNode,SIGNAL(emitUpdateTreeWidget(QString,QString,QString)),this,SLOT(updateTreeWidget(QString,QString,QString)));
    connect(commNode,SIGNAL(emitClearTreeWidget()),this,SLOT(clearTreeWidget()));
}

void MainWindow::traverseTree(QTreeWidgetItem *parentItem, QString name, QString message, QString level) {
    if(parentItem->text(0)==name) {
        // parentItem->setText(0, name);
        parentItem->setText(1, message);
        if(level=="OK"){
            parentItem->setIcon(0,QIcon(":/icon/images/ok.png"));
        } else if(level=="WARN"){
            parentItem->setIcon(0,QIcon(":/icon/images/warn.png"));
        } else if(level=="ERROR"){
            parentItem->setIcon(0,QIcon(":/icon/images/error.png"));
        } else if(level=="STALE"){
            parentItem->setIcon(0,QIcon(":/icon/images/stale.png"));
        }
        return;
    } else {
        // Recursively traverse through the children of the current item
        for (int i = 0; i < parentItem->childCount(); ++i) {
            QTreeWidgetItem *childItem = parentItem->child(i);
            traverseTree(childItem,name,message,level);
        }
    }
    
}

void MainWindow::updateTreeWidget(QString name, QString message, QString level){

     // Set the invisible root item as the parent for other items
    QTreeWidgetItem *invisibleRoot = ui->tree_diag->invisibleRootItem();
    QStringList nameList = name.split('/');
    nameList.pop_front();

    QList<QTreeWidgetItem *> foundItems = ui->tree_diag->findItems(nameList.last(), Qt::MatchContains | Qt::MatchRecursive, 0);

    // Check if any items were found
    if (foundItems.isEmpty()) {
        // If no items are found, add a new child item to the tree
        QTreeWidgetItem *newItem = new QTreeWidgetItem();
        newItem->setText(0, nameList.last());
        if(nameList.size() > 1){
            QList<QTreeWidgetItem *> foundItems2 = ui->tree_diag->findItems(nameList.at(nameList.size() - 2), Qt::MatchContains | Qt::MatchRecursive, 0);
            foundItems2.at(0)->addChild(newItem);

        } else {
            ui->tree_diag->addTopLevelItem(newItem);
        }
        
    } else {
        // Process the found items
        foreach (QTreeWidgetItem *item, foundItems) {
            item->setText(1, message);
            if(level=="OK"){
                item->setIcon(0,QIcon(":/icon/images/ok.png"));
            } else if(level=="WARN"){
                item->setIcon(0,QIcon(":/icon/images/warn.png"));
            } else if(level=="ERROR"){
                item->setIcon(0,QIcon(":/icon/images/error.png"));
            } else if(level=="STALE"){
                item->setIcon(0,QIcon(":/icon/images/stale.png"));
            }
        }
    }

    // traverseTree(invisibleRoot,nameList.last(),message,level);

    ui->tree_diag->resizeColumnToContents(0);
}

void MainWindow::clearTreeWidget(){
    ui->tree_diag->clear();
}

MainWindow::~MainWindow()
{
    delete ui;
}

