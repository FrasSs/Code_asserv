/*
 * IncFile1.h
 *
 * Created: 27/09/2012 19:33:24
 *  Author: Léon
 */ 

#ifndef TRAPEZE_POLAIRE_H_
#define TRAPEZE_POLAIRE_H_
	
/* gestion de la vitesse
Timothee Marescot
*/

#include "../hardware/leds.h" // Pour le debug
#include "struct_Commande.h"

//////////////////////////////////////////////////////////////////////////
// Variable extern

extern Commande position;
extern Commande Ordre_actuel;
extern double distance_restante;
extern double angle_restant;
extern int positionnement_precis_U;
extern int positionnement_precis_T;

//////////////////////////////////////////////////////////////////////////


void Calcule_deplacement(int Action);



#endif /* TRAPEZE_POLAIRE_H_ */ 