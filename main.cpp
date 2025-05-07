/* Copyright (C) 2025 Tatsuya Nakamura - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the LGPL 2.1 license.
 */
 
#include <QApplication>
#include <iostream>

#include "tnWindow.h"

using namespace std;

int main(int argc, char *argv[])
{
    
    QApplication app(argc, argv);
    TnWindow window;
    window.show();
    return app.exec();
}
