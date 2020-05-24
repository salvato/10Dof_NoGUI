#include <mainwindow.h>


int
main(int argc, char *argv[]) {
    MainWindow app(argc, argv);

    QCoreApplication::setOrganizationDomain("Gabriele.Salvato");
    QCoreApplication::setOrganizationName("Gabriele.Salvato");
    QCoreApplication::setApplicationName("10DOF_NoGUI");
    QCoreApplication::setApplicationVersion("0.1");

    return app.exec();
}
