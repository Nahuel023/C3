#ifndef MAPS_H
#define MAPS_H

#include <QMainWindow>

namespace Ui {
class Maps;
}

class Maps : public QMainWindow
{
    Q_OBJECT

public:
    explicit Maps(QWidget *parent = nullptr);
    ~Maps();

private:
    Ui::Maps *ui;
};

#endif // MAPS_H
