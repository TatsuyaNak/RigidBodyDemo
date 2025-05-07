/* Copyright (C) 2025 Tatsuya Nakamura - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the LGPL 2.1 license.
 */
//
// Implementation of the UIs.
//

#include <QtGui>

#include "tnGLwidget.h"
#include "tnWindow.h"

TnWindow::TnWindow()
{
    setupUi();

    QTimer *timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(idle()));
    timer->start(5);

    setWindowTitle(tr("Tatsuya Nakamura Demo"));
}

void TnWindow::writeInstruction()
{
    textEditInstruction->setText("1. \"Select File\" to select parameters file with the file dialog.\n");
    textEditInstruction->append("2. Rotate the 3D view with dragging the mouse to see six rigid bodies dangling.\n");
    textEditInstruction->append("3. Change simulation parameters with the controls below.\n");
    textEditInstruction->append(" - \"Wind\" slider to changes the strength of the wind.\n");
    textEditInstruction->append(" - Toggle \"Body Springs\" and \"World Springs\" to connect/disconnect the bodies with internal (blue dots) and extrenal (purple dots) connections respectively.\n");
    textEditInstruction->append("4. \"Restart\" button to reset the parameters.\n Enjoy!");
}

void TnWindow::setupUi()
{
    resize(1000, 800);
    pushButtonFile = new QPushButton(this);
    pushButtonFile->setText("Select File");
    pushButtonFile->setGeometry(QRect(800, 420, 120, 32));
    
    lineEditFile = new QLineEdit(this);
    lineEditFile->setGeometry(QRect(810, 390, 160, 21));
    lineEditFile->setReadOnly(TRUE);

    textEditInstruction = new QTextEdit(this);
    textEditInstruction->setGeometry(QRect(810, 20, 180, 360));
    textEditInstruction->setReadOnly(TRUE);
    
    currentGlWidget = new TnGLWidget(this);
    currentGlWidget->setGeometry(QRect(10, 4, 768, 768));
    
    groupBox = new QGroupBox(this);
    groupBox -> setTitle("Controls");
    
    groupBox->setGeometry(QRect(810, 480, 160, 240));
    pushButtonReset = new QPushButton(groupBox);
    pushButtonReset->setText("Restart");
    pushButtonReset->setGeometry(QRect(10, 200, 140, 32));
    pushButtonReset->setDisabled(TRUE);
    
    horizontalSliderWind = new QSlider(groupBox);
    horizontalSliderWind->setGeometry(QRect(20, 60, 120, 22));
    horizontalSliderWind->setOrientation(Qt::Horizontal);
    horizontalSliderWind->setRange(0, 5);
    
    labelWind = new QLabel(groupBox);
    labelWind->setText("Wind");
    labelWind->setGeometry(QRect(50, 40, 41, 16));
    
    buttonBodySprings = new QPushButton(groupBox);
    buttonBodySprings->setText("Cut Body Springs");
    buttonBodySprings->setGeometry(QRect(20, 110, 120, 20));
    
    buttonWorldSprings = new QPushButton(groupBox);
    buttonWorldSprings->setText("Cut World Springs");
    buttonWorldSprings->setGeometry(QRect(20, 150, 120, 20));
    
    QObject::connect(horizontalSliderWind, SIGNAL(valueChanged(int)),
                     this, SLOT(setWindValue()));
    QObject::connect(buttonBodySprings, SIGNAL(clicked()),
                     this, SLOT(toggleBodySprings()));
    QObject::connect(buttonWorldSprings, SIGNAL(clicked()),
                     this, SLOT(toggleWorldSprings())); 
    QObject::connect(pushButtonReset, SIGNAL(clicked()),
                     this, SLOT(resetAll()));
    QObject::connect(pushButtonFile, SIGNAL(clicked()),
                     this, SLOT(selectFile()));
    paramFile.clear();
    lineEditFile->setText(paramFile);
    writeInstruction();
}

QString const TnWindow::getParamFile()
{
    return paramFile;
}

void TnWindow::selectFile()
{
    if (!currentGlWidget)
    {
        std::cerr << "ERROR in selectFile: no widget is set." << std::endl;
        return;
    }
    currentGlWidget->stopSim();
    QString fileName = QFileDialog::getOpenFileName(this,
                                                    tr("Pick parameters"), ".");
    if (!fileName.isEmpty())
    {   
        QByteArray ba = fileName.toLocal8Bit();
        const char *charFileName = ba.data();
        int result = currentGlWidget->readParamFile((char *)charFileName);
        if (result == 1)
        {
            // If the file has been successfully loaded, immediately start the simulation.
            paramFile = fileName;
            lineEditFile->setText(fileName);
            pushButtonReset->setDisabled(FALSE);
            currentGlWidget->startSim();                
            QString message = "This parameter file has been loaded.\n";
            message = message+fileName;
            QMessageBox::information(this, tr("Please confirm"), message);
            return;
        }
        else
        {
            QString message = "Failed to read this parameter file.\n";
            message = message+fileName;
            QMessageBox::warning(this, tr("Error reading file"), message);
        }
    }
    // The simulation continues if already the file has been loaded.
    if (!paramFile.isEmpty())
    {
        pushButtonReset->setDisabled(FALSE);
        currentGlWidget->startSim();                
    }
}

void TnWindow::resetAll()
{
    if (currentGlWidget)
    {
        currentGlWidget->resetAll();
        horizontalSliderWind->setValue(0);
        buttonWorldSprings->setText("Cut World Springs");
        buttonBodySprings->setText("Cut Body Springs");
    }
}

void TnWindow::toggleWorldSprings()
{
    if (currentGlWidget)
    {
        if (currentGlWidget->toggleWorldSprings())
        {
            buttonWorldSprings->setText("Cut World Springs");
        }
        else
        {
            buttonWorldSprings->setText("Connect to World");
        }
    }
}

void TnWindow::toggleBodySprings()
{
    if (currentGlWidget)
    {
        if (currentGlWidget->toggleBodySprings())
        {
            buttonBodySprings->setText("Cut Body Springs");
        }
        else
        {
            buttonBodySprings->setText("Connect Bodies");
        }
    }
}

void TnWindow::setWindValue()
{
    if (currentGlWidget)
    {
        int value = horizontalSliderWind->value();
        currentGlWidget->setWindValue(value);
    }
}

void TnWindow::idle()
{
    if (currentGlWidget)
    {
        currentGlWidget->idle();
    }
}
