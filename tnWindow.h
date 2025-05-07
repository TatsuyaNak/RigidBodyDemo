/* Copyright (C) 2025 Tatsuya Nakamura - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the LGPL 2.1 license.
 */

#ifndef WINDOW_H
#define WINDOW_H

#include <QWidget>
#include <QGroupBox>
#include <QPushButton>
#include <QSlider>
#include <QLabel>

#include <QLineEdit>
#include <QTextEdit>
#include <QFileDialog>

#include "tnGLWidget.h"

class TnGLWidget;

class TnWindow : public QWidget
{
    Q_OBJECT

public:
    TnWindow();
    QGroupBox *groupBox;
    QPushButton *pushButtonReset, *buttonBodySprings, *buttonWorldSprings;
    QSlider *horizontalSliderWind;
    QLabel *labelWind;
    QPushButton *pushButtonFile;
    QLineEdit *lineEditFile;
    QTextEdit *textEditInstruction;
    QString const getParamFile();

private slots:
    void idle();
    void setWindValue();
    void toggleBodySprings();
    void toggleWorldSprings();
    void resetAll();
    void selectFile();

private:
    QString paramFile;
    TnGLWidget *currentGlWidget;
    void setupUi();
    void writeInstruction();
};

#endif
