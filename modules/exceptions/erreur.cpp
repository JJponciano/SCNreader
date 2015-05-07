#include "erreur.h"

Erreur::Erreur(std::string const& phrase)throw(){
    this->phrase=phrase;
}
const char* Erreur::what()const throw(){
    return this->phrase.c_str();
}

Erreur::~Erreur()throw(){

}

