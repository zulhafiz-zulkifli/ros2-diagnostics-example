#include "mainwindow.h"
#include "./ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    commNode=new rclcomm();

    connect(commNode,SIGNAL(emitDiagnostics(QString)),this,SLOT(updateDiagnostics(QString)));

    connect(commNode,SIGNAL(emitUpdateTreeWidget(QString,QString,QString)),this,SLOT(updateTreeWidget(QString,QString,QString)));
    connect(commNode,SIGNAL(emitClearTreeWidget()),this,SLOT(clearTreeWidget()));
    

}

void MainWindow::updateTreeWidget(QString name, QString message, QString level){
    // ui->tree_diag->clear();

     // Set the invisible root item as the parent for other items
    QTreeWidgetItem *invisibleRoot = ui->tree_diag->invisibleRootItem();

    QStringList nameList = name.split('/');
    nameList.pop_front();
    int treeLevel = nameList.size();

    
    // std::cout << nameList.last().toStdString() << std::endl;
    
    if(treeLevel==1){
        QTreeWidgetItem *item1 = new QTreeWidgetItem();
        item1->setText(0, nameList.last());
        item1->setText(1, message);
        if(level=="OK"){
            item1->setIcon(0,QIcon(":/icon/images/ok.png"));
        } else if(level=="WARN"){
            item1->setIcon(0,QIcon(":/icon/images/warn.png"));
        } else if(level=="ERROR"){
            item1->setIcon(0,QIcon(":/icon/images/error.png"));
        } else if(level=="STALE"){
            item1->setIcon(0,QIcon(":/icon/images/stale.png"));
        }
        

        invisibleRoot->addChild(item1);
        // item1->setIcon(0, util.level_to_icon(status.level))
    } else {
        // Check if there are any children
        if (invisibleRoot->childCount() > 0) {
            // Get the last child
            QTreeWidgetItem* lastChild = invisibleRoot->child(invisibleRoot->childCount() - 1);

            QTreeWidgetItem *item1 = new QTreeWidgetItem();
            item1->setText(0, nameList.last());
            item1->setText(1, message);
            if(level=="OK"){
                item1->setIcon(0,QIcon(":/icon/images/ok.png"));
            } else if(level=="WARN"){
                item1->setIcon(0,QIcon(":/icon/images/warn.png"));
            } else if(level=="ERROR"){
                item1->setIcon(0,QIcon(":/icon/images/error.png"));
            } else if(level=="STALE"){
                item1->setIcon(0,QIcon(":/icon/images/stale.png"));
            }
            

            lastChild->addChild(item1);
            }
    }


    // // Add items to the tree
    // QTreeWidgetItem *item1 = new QTreeWidgetItem(invisibleRoot);
    // item1->setText(0, "Item 1");
    // item1->setText(1, "File");
    // item1->setText(2, "10 KB");

    // QTreeWidgetItem *item2 = new QTreeWidgetItem(invisibleRoot);
    // item2->setText(0, "Item 2");
    // item2->setText(1, "Folder");

    // QTreeWidgetItem *subItem1 = new QTreeWidgetItem(item2);
    // subItem1->setText(0, "Subitem 1");
    // subItem1->setText(1, "File");
    // subItem1->setText(2, "5 KB");

    // QTreeWidgetItem *subItem2 = new QTreeWidgetItem(item2);
    // subItem2->setText(0, "Subitem 2");
    // subItem2->setText(1, "File");
    // subItem2->setText(2, "8 KB");

    ui->tree_diag->expandAll();
    ui->tree_diag->resizeColumnToContents(0);

}

void MainWindow::clearTreeWidget(){
    ui->tree_diag->clear();
}

void MainWindow::updateDiagnostics(QString status){

    // QFont font1 = ui->display_status->font();
    // font1.setPointSize(17);
    // ui->display_status->setFont(font1);
    // ui->display_status->clear();

    // ui->display_status->setText(status);

    // if (status != "INITIALIZING" && !initialized_) {
    //     enableButtons(true);
    //     initialized_ = true;
    // }

    // // Set text color based on status
    // if (status == "EMERGENCY") {
    //     ui->display_status->setStyleSheet("color: red;");
    // } else if (status == "IDLE") {
    //     ui->display_status->setStyleSheet("color: black;");
    // } else if (status == "INITIALIZING") {
    //     ui->display_status->setStyleSheet("color: blue;");
    // } else {
    //     ui->display_status->setStyleSheet("color: green;");
    // }
    
    // ui->display_status->setAlignment(Qt::AlignCenter);
}

MainWindow::~MainWindow()
{
    delete ui;
}

