#ifndef SCNREADER_VIEW_H
#define SCNREADER_VIEW_H

#include <QObject>
#include <QWidget>
#include "../modules/pcl/view/view_pcl.h"
#include "scnreader_model.h"
class scnreader_view : public View_pcl
{
     Q_OBJECT
public:
    scnreader_view(QWidget* parent = 0);
    ~scnreader_view();
     virtual void paintGL();
     void loadCloudFromTXT();
     void loadFromSCN();
private:
    scnreader_model scnreaderFond;
};

#endif // SCNREADER_VIEW_H
