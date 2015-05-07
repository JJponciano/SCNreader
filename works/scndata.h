#ifndef SCNDATA_H
#define SCNDATA_H


#include "datapackage.h"
class ScnData
{
public:
    ScnData();
    ScnData(std::string pathname);
    ~ScnData();


    /**
     * @brief loadFromSCN
     * @param pathname the path of the load file
     * @return the cloud loaded
     */
    void  loadFromSCN(std::string pathname);

    QVector<Datapackage> getPackages() const;
    void setPackages(const QVector<Datapackage> &value);

    QByteArray getAlldatas() const;
    void setAlldatas(const QByteArray &value);

    int getIndexreader() const;
    void setIndexreader(int value);

private:
    void assigningDatas();
    QVector<Datapackage>packages;
    QByteArray alldatas;
    int indexreader;

};

#endif // SCNDATA_H
