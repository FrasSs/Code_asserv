#ifndef ORDRE
#define ORDRE

#include "struct_Commande.h"

extern Commande Ordre_actuel;
extern Commande Ordre_suivant;
extern Commande* Tab_ordre[];
extern uint8_t etat;
extern uint8_t new_etat;
extern int nb_ordre;

void Actualisation_Ordre(int stack);

void fct_Ordre_suivant(int * cond1,int *cond2,int stack,int stop);

#endif