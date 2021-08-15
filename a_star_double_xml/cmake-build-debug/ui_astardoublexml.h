/********************************************************************************
** Form generated from reading UI file 'astardoublexml.ui'
**
** Created by: Qt User Interface Compiler version 4.8.7
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_ASTARDOUBLEXML_H
#define UI_ASTARDOUBLEXML_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QHeaderView>
#include <QtGui/QMainWindow>
#include <QtGui/QMenuBar>
#include <QtGui/QPushButton>
#include <QtGui/QStatusBar>
#include <QtGui/QToolBar>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_AStarDoubleXML
{
public:
    QWidget *centralWidget;
    QPushButton *Pathplan;
    QPushButton *pushButton;
    QPushButton *OutputTxt;
    QPushButton *ReadFile;
    QPushButton *ReadTaskFile;
    QMenuBar *menuBar;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *AStarDoubleXML)
    {
        if (AStarDoubleXML->objectName().isEmpty())
            AStarDoubleXML->setObjectName(QString::fromUtf8("AStarDoubleXML"));
        AStarDoubleXML->resize(279, 205);
        centralWidget = new QWidget(AStarDoubleXML);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        Pathplan = new QPushButton(centralWidget);
        Pathplan->setObjectName(QString::fromUtf8("Pathplan"));
        Pathplan->setGeometry(QRect(150, 30, 80, 26));
        pushButton = new QPushButton(centralWidget);
        pushButton->setObjectName(QString::fromUtf8("pushButton"));
        pushButton->setGeometry(QRect(150, 100, 101, 31));
        OutputTxt = new QPushButton(centralWidget);
        OutputTxt->setObjectName(QString::fromUtf8("OutputTxt"));
        OutputTxt->setGeometry(QRect(150, 60, 80, 26));
        ReadFile = new QPushButton(centralWidget);
        ReadFile->setObjectName(QString::fromUtf8("ReadFile"));
        ReadFile->setGeometry(QRect(21, 31, 80, 26));
        ReadTaskFile = new QPushButton(centralWidget);
        ReadTaskFile->setObjectName(QString::fromUtf8("ReadTaskFile"));
        ReadTaskFile->setGeometry(QRect(21, 63, 82, 26));
        AStarDoubleXML->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(AStarDoubleXML);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 279, 23));
        AStarDoubleXML->setMenuBar(menuBar);
        mainToolBar = new QToolBar(AStarDoubleXML);
        mainToolBar->setObjectName(QString::fromUtf8("mainToolBar"));
        AStarDoubleXML->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(AStarDoubleXML);
        statusBar->setObjectName(QString::fromUtf8("statusBar"));
        AStarDoubleXML->setStatusBar(statusBar);

        retranslateUi(AStarDoubleXML);

        QMetaObject::connectSlotsByName(AStarDoubleXML);
    } // setupUi

    void retranslateUi(QMainWindow *AStarDoubleXML)
    {
        AStarDoubleXML->setWindowTitle(QApplication::translate("AStarDoubleXML", "AStarDoubleXML", 0, QApplication::UnicodeUTF8));
        Pathplan->setText(QApplication::translate("AStarDoubleXML", "PathPlan", 0, QApplication::UnicodeUTF8));
        pushButton->setText(QApplication::translate("AStarDoubleXML", "TestRePlanning", 0, QApplication::UnicodeUTF8));
        OutputTxt->setText(QApplication::translate("AStarDoubleXML", "OutputTxt", 0, QApplication::UnicodeUTF8));
        ReadFile->setText(QApplication::translate("AStarDoubleXML", "ReadFile", 0, QApplication::UnicodeUTF8));
        ReadTaskFile->setText(QApplication::translate("AStarDoubleXML", "ReadTaskFile", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class AStarDoubleXML: public Ui_AStarDoubleXML {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_ASTARDOUBLEXML_H
