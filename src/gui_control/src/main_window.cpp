
/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/gui_control/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace gui_control {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
	, qnode(argc,argv)
{
	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

    ReadSettings();
	setWindowIcon(QIcon(":/images/icon.png"));
	ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
	QObject::connect(ui.button_apply, SIGNAL(clicked(bool check)), this, SLOT(on_button_apply_clicked(bool check)));
	QObject::connect(ui.mode0, SIGNAL(clicked(bool check)), this, SLOT(on_mode0_clicked(bool check)));
	QObject::connect(ui.mode1, SIGNAL(clicked(bool check)), this, SLOT(on_mode1_clicked(bool check)));
	QObject::connect(ui.mode2, SIGNAL(clicked(bool check)), this, SLOT(on_mode2_clicked(bool check)));
	QObject::connect(ui.mode3, SIGNAL(clicked(bool check)), this, SLOT(on_mode3_clicked(bool check)));
	
	
	/*********************
	** Logging
	**********************/
	ui.view_logging->setModel(qnode.loggingModel());
    QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));
 
    /*********************
    ** Connect
    **********************/


    /*********************
    ** Auto Start
    **********************/
    if ( ui.checkbox_remember_settings->isChecked() ) {
        on_button_connect_clicked(true);
    }
}

MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::showNoMasterMessage() {
	QMessageBox msgBox;
	msgBox.setText("Couldn't find the ros master.");
	msgBox.exec();
    close();
}

/*
 * These triggers whenever the button is clicked, regardless of whether it
 * is already checked or not.
 */

void MainWindow::on_button_connect_clicked(bool check ) {
	if ( ui.checkbox_use_environment->isChecked() ) {
		if ( !qnode.init() ) {
			showNoMasterMessage();
		} else {
			ui.button_connect->setEnabled(false);
		}
	} else {
		if ( ! qnode.init(ui.line_edit_master->text().toStdString(),
				   ui.line_edit_host->text().toStdString()) ) {
			showNoMasterMessage();
		} else {
			ui.button_connect->setEnabled(false);
			ui.line_edit_master->setReadOnly(true);
			ui.line_edit_host->setReadOnly(true);
			ui.line_edit_topic->setReadOnly(true);
		}
	}
}

void MainWindow::on_button_apply_clicked(bool check ){	
	
	bool error=false;
	std::string px=ui.px->text().toStdString();
	if (px.empty()){error=true;}
	else{
	for(int i = 0;i < (px.size());i++) {
		if(isdigit(px[i])) {
        
		} 	
		else {
        error=true;
        break;
		}
	}
	}
	
	std::string py=ui.py->text().toStdString();
	if (py.empty()){error=true;}
	else{
	for(int i = 0;i < (py.size());i++) {
		if(isdigit(py[i])) {
        
		} 	
		else {
        error=true;
        break;
		}
	}
	}

	std::string pz=ui.pz->text().toStdString();
	if (pz.empty()){error=true;}
	else{
	for(int i = 0;i < (pz.size());i++) {
		if(isdigit(pz[i])) {
        
		} 	
		else {
        error=true;
        break;
		}
	}
	}

	std::string rx=ui.rx->text().toStdString();
	if (rx.empty()){error=true;}
	else{
	for(int i = 0;i < (rx.size());i++) {
		if(isdigit(rx[i])) {
        
		} 	
		else {
        error=true;
        break;
		}
	}
	}

	std::string ry=ui.ry->text().toStdString();
	if (ry.empty()){error=true;}
	else{
	for(int i = 0;i < (ry.size());i++) {
		if(isdigit(ry[i]) && !ry.empty()) {
        
		} 	
		else {
        error=true;
        break;
		}
	}
	}

	std::string rz=ui.rz->text().toStdString();
	if (rz.empty()){error=true;}
	else{
	for(int i = 0;i < (pz.size());i++) {
		if(isdigit(rz[i]) && !rz.empty()) {
        
		} 	
		else {
        error=true;
        break;
		}
	}
	}
	
	if(error){
		ui.label_alert->setText("Something is not a number");
	}
	else {
		ui.vpx->setText(ui.px->text());
		ui.vpy->setText(ui.py->text());
		ui.vpz->setText(ui.pz->text());
		ui.vrx->setText(ui.rx->text());
		ui.vry->setText(ui.ry->text());
		ui.vrz->setText(ui.rz->text());
		ui.label_alert->setText("Done");
		qnode.setstate(atoi(px.c_str()),atoi(py.c_str()),atoi(pz.c_str()),atoi(rx.c_str()),atoi(ry.c_str()),atoi(rz.c_str()),0);
	}
}


void MainWindow::on_checkbox_use_environment_stateChanged(int state) {
	bool enabled;
	if ( state == 0 ) {
		enabled = true;
	} else {
		enabled = false;
	}
	ui.line_edit_master->setEnabled(enabled);
	ui.line_edit_host->setEnabled(enabled);
	//ui.line_edit_topic->setEnabled(enabled);
}


void MainWindow::on_mode0_clicked(bool check){
	qnode.setmode(0);
}

void MainWindow::on_mode1_clicked(bool check){
	qnode.setmode(1);
}

void MainWindow::on_mode2_clicked(bool check){
	qnode.setmode(2);
}

void MainWindow::on_mode3_clicked(bool check){
	qnode.setmode(3);
}

/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */
void MainWindow::updateLoggingView() {
        ui.view_logging->scrollToBottom();
}

/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void MainWindow::on_actionAbout_triggered() {
    QMessageBox::about(this, tr("About ..."),tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Yujin Robot</p><p>This package needs an about description.</p>"));
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::ReadSettings() {
    QSettings settings("Qt-Ros Package", "gui_control");
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());
    QString master_url = settings.value("master_url",QString("http://192.168.1.2:11311/")).toString();
    QString host_url = settings.value("host_url", QString("192.168.1.3")).toString();
    //QString topic_name = settings.value("topic_name", QString("/chatter")).toString();
    ui.line_edit_master->setText(master_url);
    ui.line_edit_host->setText(host_url);
    //ui.line_edit_topic->setText(topic_name);
    bool remember = settings.value("remember_settings", false).toBool();
    ui.checkbox_remember_settings->setChecked(remember);
    bool checked = settings.value("use_environment_variables", false).toBool();
    ui.checkbox_use_environment->setChecked(checked);
    if ( checked ) {
    	ui.line_edit_master->setEnabled(false);
    	ui.line_edit_host->setEnabled(false);
    	//ui.line_edit_topic->setEnabled(false);
    }
}

void MainWindow::WriteSettings() {
    QSettings settings("Qt-Ros Package", "gui_control");
    settings.setValue("master_url",ui.line_edit_master->text());
    settings.setValue("host_url",ui.line_edit_host->text());
    //settings.setValue("topic_name",ui.line_edit_topic->text());
    settings.setValue("use_environment_variables",QVariant(ui.checkbox_use_environment->isChecked()));
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());
    settings.setValue("remember_settings",QVariant(ui.checkbox_remember_settings->isChecked()));

}

void MainWindow::closeEvent(QCloseEvent *event)
{
	WriteSettings();
	QMainWindow::closeEvent(event);
}

}  // namespace gui_control

