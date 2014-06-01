/*
 * config.c
 *
 * Created: 29/09/2012 17:05:55
 *
 */ 


#include "../include/config.h"
#include "../include/Odometrie.h"

//////////////////////////////////////////////////////////////////////////
// Repère absolu :			^ Y		
//							|	
//							|		
//							|			
//							|						X
//	 -----------------------+----------------------->
//

//////////////////////////////////////////////////////////////////////////
// Variable extern 
extern int codeur_d;
extern int codeur_g;
extern double distance_U;
extern double distance_Theta;
extern Commande position;
//////////////////////////////////////////////////////////////////////////

void fct_odometrie()
{
	static double distance_U_total = 0;
	static double distance_Theta_total = 0;
	
	double codeur_d_courant,codeur_g_courant,Rayon_rotation,corde,angle_projection,relatif_X,relatif_Y;
		
	codeur_d_courant = Sens_codeur_D*(codeur_d - 10000)*INC_CODEUR_D;// /1000.0; // pas en mm
	codeur_g_courant = Sens_codeur_G*(codeur_g - 10000)*INC_CODEUR_G;// /1000.0; // pas en mm
		
	// mise à jour de la position du robot
	distance_U = (codeur_g_courant*(ANTRAXE_D/(ANTRAXE_D+ANTRAXE_G)))+(codeur_d_courant*(ANTRAXE_G/(ANTRAXE_D+ANTRAXE_G)));  // distance parcouru
	distance_Theta = ((codeur_d_courant-codeur_g_courant)/(ANTRAXE_D+ANTRAXE_G));//*1000; // angle parcouru
	
	distance_U_total += distance_U;
	distance_Theta_total += distance_Theta;
			
	if(distance_Theta>=((5000*M_PI)/(100000*180))) //seuil à 5*10^-2 degré // seuil à changer si on passe en rad/s car on est actuelement en rad/10ms
	{// cas ou il y a un arc de cercle prononcé
		Rayon_rotation=distance_U/(distance_Theta); // rayon du cercle tracé par le déplacement
		corde=2*Rayon_rotation*sin(distance_Theta/2); // calcule de la corde du l'arc de cercle au centre du robot
		angle_projection=(distance_Theta/2)+position.Theta; // angle permettant la projection de la corde sur le repère fixe au sol
	}
	else
	{// cas ou on fais l'aproximation que la trajectoire à effectuer est purement une droite la corde et la distance parcourue sont les même
		angle_projection=(distance_Theta/2)+position.Theta; // angle permettant la projection de la corde sur le repère fixe au sol
		corde=distance_U;
	}
	
	// déplacement relatif
	relatif_X=-sin(angle_projection)*corde;
	relatif_Y=cos(angle_projection)*corde;
				
	// position absolu
	position.Theta= fmod(position.Theta + distance_Theta,2*M_PI); // position précédente à laquelle on ajoute la distance parcouru
		
	position.X = position.X + relatif_X;
	position.Y = position.Y + relatif_Y;
}


