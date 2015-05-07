#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "modules/pcl/view/view_pcl.h"
#include <QMessageBox>
#include "works/scnreader_view.h"
#include "works/vueparetape.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    virtual void keyPressEvent( QKeyEvent *keyEvent );
    virtual void mousePressEvent(QMouseEvent *event);
    virtual void mouseReleaseEvent(QMouseEvent *event);
    virtual void mouseMoveEvent(QMouseEvent *event);
public slots:

    void clear();
    void extract();
    void planarSeg();
    void affichageOK();
    void loadFromFile();
    void loadFromSCN();
    void loadCloudFromTXT();
    void saveFromFile();
    void saveFromFileTXT();
    void changeD();
    void changeF();
private:
    Ui::MainWindow *ui;
    VueParEtape *qw;
};

#endif // MAINWINDOW_H
