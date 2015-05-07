#ifndef ERREUR_H
#define ERREUR_H
/**
 * @file erreur.h
 * @brief file to the managements of exeptions
 * @author Jean-Jacques PONCIANO
 * @version 0.1
 */

#include <exception>
#include <string>

/**
 * @class Erreur
 * @brief The Erreur class  for manage exeptions
 * This class is used for throw a custom exeption
 *
 * For example, if you want throw a exeption and catch it:
 *
 * @code
 *
 * try{
 *
 * throw Erreur("Description of exeption");
 *
 * }catch(std::exception const& e){
 *  QMessageBox::critical(this, "Error", e.what());
 * }
 * @endcode
 *
 *
 */
class Erreur : public std::exception
{
public:
    /**
     * @brief Erreur create exeption
     * @param phrase sentence explaining the reasons for the throwing of exceptions
     */
    Erreur(std::string const& phrase="error detected")throw();
    virtual  ~Erreur()throw(); //Destructeur.
    virtual const char* what()const throw();

private:
    std::string phrase; ///< Description of exeption
};

#endif // ERREUR_H
