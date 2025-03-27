/********************************************************************************
** Form generated from reading UI file 'MainWindow.ui'
**
** Created by: Qt User Interface Compiler version 5.14.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralwidget;
    QVBoxLayout *verticalLayout;
    QHBoxLayout *horizontalLayout;
    QPushButton *connect;
    QPushButton *disconnect;
    QSpacerItem *horizontalSpacer;
    QGroupBox *groupBox;
    QHBoxLayout *horizontalLayout_2;
    QPushButton *ping;
    QSpinBox *id;
    QLabel *status;
    QSpacerItem *verticalSpacer;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(396, 178);
        centralwidget = new QWidget(MainWindow);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        verticalLayout = new QVBoxLayout(centralwidget);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        connect = new QPushButton(centralwidget);
        connect->setObjectName(QString::fromUtf8("connect"));

        horizontalLayout->addWidget(connect);

        disconnect = new QPushButton(centralwidget);
        disconnect->setObjectName(QString::fromUtf8("disconnect"));
        disconnect->setEnabled(false);

        horizontalLayout->addWidget(disconnect);

        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout->addItem(horizontalSpacer);


        verticalLayout->addLayout(horizontalLayout);

        groupBox = new QGroupBox(centralwidget);
        groupBox->setObjectName(QString::fromUtf8("groupBox"));
        horizontalLayout_2 = new QHBoxLayout(groupBox);
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        ping = new QPushButton(groupBox);
        ping->setObjectName(QString::fromUtf8("ping"));

        horizontalLayout_2->addWidget(ping);

        id = new QSpinBox(groupBox);
        id->setObjectName(QString::fromUtf8("id"));
        id->setMinimumSize(QSize(60, 0));
        id->setMinimum(1);
        id->setMaximum(15);

        horizontalLayout_2->addWidget(id);

        status = new QLabel(groupBox);
        status->setObjectName(QString::fromUtf8("status"));
        QSizePolicy sizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
        sizePolicy.setHorizontalStretch(1);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(status->sizePolicy().hasHeightForWidth());
        status->setSizePolicy(sizePolicy);

        horizontalLayout_2->addWidget(status);


        verticalLayout->addWidget(groupBox);

        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout->addItem(verticalSpacer);

        MainWindow->setCentralWidget(centralwidget);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QCoreApplication::translate("MainWindow", "MainWindow", nullptr));
        connect->setText(QCoreApplication::translate("MainWindow", "Connect", nullptr));
        disconnect->setText(QCoreApplication::translate("MainWindow", "Disconnect", nullptr));
        groupBox->setTitle(QCoreApplication::translate("MainWindow", "Ping", nullptr));
        ping->setText(QCoreApplication::translate("MainWindow", "Ping", nullptr));
        status->setText(QCoreApplication::translate("MainWindow", "Waiting...", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
