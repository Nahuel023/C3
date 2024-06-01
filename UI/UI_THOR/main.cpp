
#include "qform.h"

#include <QApplication>


int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    QForm w;
    w.show();
    return a.exec();
}
