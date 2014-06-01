#ifndef ORDRE
#define ORDRE

#include "struct_Commande.h"
#include "Ordres.h"

extern Commande* Tab_ordre[];
extern uint8_t etat;		// num�ro de l'ordre en cour
extern uint8_t new_etat;	// num�ro du dernier ordre re�u

extern double moteur_G;
extern double moteur_D;
extern double Vitesse_C_U;
extern double Vitesse_C_T;
extern double distance_restante;
extern double angle_restant;
extern int positionnement_precis_U;
extern int positionnement_precis_T;

void fct_STOP_F(int stop);

#endif