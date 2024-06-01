#ifndef DEBUGGER_H
#define DEBUGGER_H

#include <QMainWindow>

namespace Ui {
class debugger;
}

class debugger : public QMainWindow
{
    Q_OBJECT

public:
    explicit debugger(QWidget *parent = nullptr);
    ~debugger();

private:
    Ui::debugger *ui;
};

#endif // DEBUGGER_H
