/**
 * @file erreur.h
 * @brief file to the managements of exeptions
 * @copyright 2015 Jean-Jacques PONCIANO, Claire PRUDHOMME
 * All rights reserved.
 * This file is part of scn reader.
 *
 * scn reader is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * scn reader is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Foobar.  If not, see <http://www.gnu.org/licenses/>
 * @author Jean-Jacques PONCIANO and Claire PRUDHOMME
 * Contact: ponciano.jeanjacques@gmail.com
 * @version 0.1
 */
#ifndef ERREUR_H
#define ERREUR_H

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
