#include "debugger.h"
#include "ui_debugger.h"

debugger::debugger(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::debugger)
{
    ui->setupUi(this);
}

debugger::~debugger()
{
    delete ui;
}
