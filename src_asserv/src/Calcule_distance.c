/*
 * CFile1.c
 *
 * Created: 27/09/2012 19:33:14
 *  Author : Léon
 *	Corrigé : François
 */ 


#include <math.h>

#include "../include/Calcule_distance.h"
#include "../include/config.h"

/*
 * gestionTrajectoire.c
 *
 *  Created on: 23 sept. 2012
 *      Author: Manuel
 */

void Calcule_deplacement(int action)
{

	// calcule sens //////////////////////////////////////////////////////////
	
	double X_Bp=0,Y_Bp=0,X_Bpp=0,Y_Bpp=0,Theta_B=0,Theta_Bp=0;
	signed int sens_U=0,sens_T=0;
	
	//////////////////////////////////////////////////////////////////////////
	// Distance signé en fonction de la direction//////////////////////////////
	
	//calcule pour déterminer si le point d'arrivé est derrière le robot (négative en y sur le repère fixe au robot) 
	X_Bp=(Ordre_actuel.X)-position.X;
	Y_Bp=(Ordre_actuel.Y)-position.Y;
	
	distance_restante = sqrt((X_Bp)*(X_Bp) + (Y_Bp)*(Y_Bp) );
	
	Theta_B=-fmod(atan2(X_Bp,Y_Bp),2*M_PI);
	Theta_Bp=fmod(Theta_B-position.Theta,2*M_PI);
	
	Y_Bpp=distance_restante*cos(Theta_Bp); // coordonné du point arrivé sur le repère fixe au robot
	X_Bpp=-distance_restante*sin(Theta_Bp); // coordonné du point arrivé sur le repère fixe au robot	
	
	if (Y_Bpp<0) // si le point est derrière le robot
	{
		sens_U=-1; // la fct fais passer une distance négative pour ordonner un recul du robot
	}
	else
	{
		sens_U=1; // la fct fais passer une distance négative pour ordonner une avance du robot
	}
	
	//////////////////////////////////////////////////////////////////////////
	//angle optimale en passant les angles sur -PI et PI//////////////////////

	switch (action)
	{
		case AVANCE:
		{
			if (distance_restante<20)
			{
				distance_restante= Y_Bpp;
				
				angle_restant = fmod(Ordre_actuel.Theta-position.Theta,2.0*M_PI); // angle à parcourir pour arriver à l'angle désiré sur le repère fixe au robot
			}
			else
			{
				distance_restante *= sens_U;
				
				angle_restant = fmod(Theta_Bp,2.0*M_PI); // angle à parcourir pour arriver à l'angle désiré sur le repère fixe au robot
			}

			if (angle_restant>=M_PI)
			{
				sens_T=-1;
			}

			if (angle_restant<=(-M_PI))
			{
				sens_T=1;
			}

			angle_restant+=sens_T*(2*M_PI);

			//////////////////////////////////////////////////////////////////////////
			// Retour de l'information d'arrivée au point désiré//////////////////////

			if(fabs(angle_restant)<(M_PI/100.0)) // arrèt des moteurs quand le robot est à la position Theta demandée
			{
				positionnement_precis_T=1;
			}
			else
			{
				positionnement_precis_T=0;
			}
			
			if(fabs(distance_restante)<10) // arrèt des moteurs quand le robot est à la position x et y demandée
			{
				positionnement_precis_U= 1;
			}
			else
			{
				//////////////////////////////////////////////////////////////////////////
				// en cas de distance Théta trop grande, stoper U pour règler seulement theta
				if ((abs(angle_restant)>(M_PI/8.0))&&(distance_restante>30))
				{
					positionnement_precis_U= 1;
				}
				else
				{
					
					positionnement_precis_U=0;
				}
				//////////////////////////////////////////////////////////////////////////
				
			}
			break;
		}
		
		
		case AVANCE_Free:
		{
			distance_restante=Y_Bpp;// *= sens_U;
			
			if (distance_restante>0)
			{
				angle_restant = fmod(Theta_Bp,2.0*M_PI); // angle à parcourir pour arriver à l'angle désiré sur le repère fixe au robot
			}
			else
			{
				angle_restant = fmod(Theta_Bp,2.0*M_PI)-M_PI; // angle à parcourir pour arriver à l'angle désiré sur le repère fixe au robot
			}
			
			if (angle_restant>M_PI)
			{
				sens_T=-1;
			}
			
			if (angle_restant<(-M_PI))
			{
				sens_T=1;
			}
			
			angle_restant+=sens_T*(2.0*M_PI);
			
			//////////////////////////////////////////////////////////////////////////
			// Retour de l'information d'arrivée au point désiré//////////////////////

			if(fabs(angle_restant)<(M_PI/15.0)) // arrèt des moteurs quand le robot est à la position Theta demandée
			{
				positionnement_precis_T=  1;
			}
			else
			{
				positionnement_precis_T=0;
			}
			
			if((fabs(distance_restante)<40)/*||(fabs(angle)>(M_PI_4))*/) // arrèt des moteurs quand le robot est à la position x et y demandée
			{
				positionnement_precis_U= 1;
				/*if (fabs(distance)<40)
				{
					*arret_T= 1;
				}
				else
				{
					*arret_T= 0;
				}*/
			}
			else
			{
				
				positionnement_precis_U=0;
			}
			
			break;
		}
		case TOURNE:
		{
			distance_restante= Y_Bpp;
			
			angle_restant = fmod(Ordre_actuel.Theta-position.Theta,2.0*M_PI); // angle à parcourir pour arriver à l'angle désiré sur le repère fixe au robot
			
			if (angle_restant>=M_PI)
			{
				sens_T=-1;
			}

			if (angle_restant<=(-M_PI))
			{
				sens_T=1;
			}

			angle_restant+=sens_T*(2.0*M_PI);
			
			//////////////////////////////////////////////////////////////////////////
			// Retour de l'information d'arrivée au point désiré//////////////////////
			
			if(fabs(distance_restante)<5) // arrèt des moteurs quand le robot est à la position x et y demandée
			{
				positionnement_precis_U= 1;
			}
			else
			{
				positionnement_precis_U=0;
			}

			if(fabs(angle_restant)<(M_PI/100.0)) // arrèt des moteurs quand le robot est à la position Theta demandée
			{
				positionnement_precis_T= 1;
				positionnement_precis_U= 1;
			}
			else
			{
				positionnement_precis_T=0;
			}
			break;
		}	
	}
}