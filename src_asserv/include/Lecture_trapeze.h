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

double calculTrapez (int action,double Vitesse,double V_Min,double V_Max, double Dist, double A_Desc,double A_Acc,int *Arrive);
//									|||							|||				|||				|||				|||			|||				|||	Distance utiliser en fin du Trapez pour la précision du positionnement
//									|||							|||				|||				|||				||| 		||| Vitesse d'Accélération => une constance que l'on peut régler
//									|||							|||				|||				|||				|||	Vitesse de Déscélération => une constance que l'on peut régler
//									|||							|||				|||				|||	Distance parcourue
//									|||							|||				|||
//									|||							|||				||| Vitesse de la phase de mise en position précise
//									|||							||| Vitesse macimal	
//									|||	
//									||| Vitesse actuelle
	/*
	Fonction qui gere la vitesse du robot suivant un trapeze : acceleration lineaire (ACCELERATION), puis maintient a la vitesse max (Vmax),
	puis deceleration lineaire avant d'arriver a la position finale. La distance trapez fixe la distance, par rapport a la position finale,
	a partir de laquelle le robot commencera a ralentir.
	*/

#endif /* TRAPEZE_POLAIRE_H_ */ 