/*
 * main.c
 *
 * Created: 18/09/2012 16:26:30
 *  Author: Manuel
 *		Note le minimum pour faire tourné des moteurs.FONCTIONNE
 */ 

#define F_CPU 32000000UL

#include <util/delay.h>
#include <stdint.h>
#include <math.h>
#include "./include/config.h"
#include "./hardware/avr_compiler.h"

//////////////////////////////////////////////////////////////////////////
// I2C ///////////////////////////////////////////////////////////////////

extern uint8_t I2CNewOrderFlag;
//extern uint8_t I2CNewOrderAkwnoledged;
Commande I2CNewOrder;
i2c_packet i2c_envoie;

extern int Com_i2c;

//////////////////////////////////////////////////////////////////////////
// Constante de temps ////////////////////////////////////////////////////
volatile static unsigned char goAsserv=0;

//////////////////////////////////////////////////////////////////////////
// Odométrie /////////////////////////////////////////////////////////////

int codeur_g = 10000; // Variable contrôlé grace au timer dont l'horloge et une roue codeuse
int codeur_d = 10000; // Variable contrôlé grace au timer dont l'horloge et une roue codeuse

Commande position={0,0,0,0};

double distance_Theta = 0.0;
double distance_U = 0.0;	// Vitesse Réelle = vitesse_U*10 (donne des m/s) car vitesse_U et en mm/0.01s.


//////////////////////////////////////////////////////////////////////////
// Ordres ////////////////////////////////////////////////////////////////

#define STACK_SIZE 4 // taille de la pile utilisé
Commande Ordre1={STOP_F,0,0,0};
Commande Ordre2={STOP_F,0,0,0};
Commande Ordre3={STOP_F,0,0,0};
Commande Ordre4={STOP_F,0,0,0};
Commande End_ordre={0,0,0,0};
Commande* Tab_ordre[] = {&Ordre1 , &Ordre2, &Ordre3, &Ordre4,&End_ordre};
Commande Ordre_actuel;
Commande Ordre_suivant;

uint8_t etat = 0;		// numéro de l'ordre en cour
uint8_t new_etat = 0;	// numéro du dernier ordre reçu
int nb_ordre;			// nombre d'ordre enregistré

//////////////////////////////////////////////////////////////////////////
// Commance //////////////////////////////////////////////////////////////
// Vitesse Commande
double Vitesse_C_U = 0.0 ;
double Vitesse_C_T = 0.0 ;

// Trapeze de vitesse
double distance_restante=0.0;
double angle_restant=0.0;

// Ralentissement pour l'action AVANCE_Free
double V_Min=0.0;

//arrivé à l'endoit désiré
int positionnement_precis_U=0;
int positionnement_precis_T=0;

// PID 
double PID_V_U=0.0; 
double PID_V_T=0.0;

// Blocage
int BugBloquage=0;

// Moteur
double moteur_G = 0.0;
double moteur_D = 0.0;


//////////////////////////////////////////////////////////////////////////
// Varriable qui permet de dire si le robot avance ou non. ///////////////
//(fonction blocage)//////////////////////////////////////////////////////

//uint8_t libre = 1; //utilisé ln82 dans traitementI2C.c

//////////////////////////////////////////////////////////////////////////
//Test
#define TEST_CODEUSES 0

//////////////////////////////////////////////////////////////////////////


int main(void)
{
//////////////////////////////////////////////////////////////////////////
	init();
	

#if TEST_CODEUSES
	int tot_codeur_G = 0;
	int tot_codeur_D = 0;
	
	int tot_codeur_G_2 = 0;
	int tot_codeur_D_2 = 0;
		/*
	Ordre1.Type = AVANCE;
	Ordre1.X = 0.0;
	Ordre1.Y = 600.0;
	Ordre1.Theta = -M_PI_2;
	
	Ordre2.Type = AVANCE;
	Ordre2.X = -900.0;
	Ordre2.Y = 600.0;
	Ordre2.Theta = -M_PI_2;
	
	Ordre3.Type = AVANCE;
	Ordre3.X = -1500.0;
	Ordre3.Y = 600.0;
	Ordre3.Theta = 0;
	
	Ordre4.Type = AVANCE;
	Ordre4.X = -1500.0;
	Ordre4.Y = 0.0;
	Ordre4.Theta = 0;
	
	new_etat=4;
		*/
	
//Carré pour test AVANCE /////////////////////////////////////////////////

	/*Ordre1.Type = AVANCE;
	Ordre1.X = 0.0;
	Ordre1.Y = 200.0;
	Ordre1.Theta = -M_PI_2;
		
	Ordre2.Type = AVANCE;
	Ordre2.X = 200.0;
	Ordre2.Y = 200.0;
	Ordre2.Theta = -M_PI;
		
	Ordre3.Type = AVANCE;
	Ordre3.X = 200.0;
	Ordre3.Y = 0.0;
	Ordre3.Theta = M_PI_2;
		
	Ordre4.Type = AVANCE;
	Ordre4.X = 0.0;
	Ordre4.Y = 0.0;
	Ordre4.Theta = 0;
	
//Arc de cerlce pour tester AVANCE_Free////////////////////////////////////

	Ordre1.Type = AVANCE_Free;
	Ordre1.X = 50.0;
	Ordre1.Y = 150.0;
	Ordre1.Theta = -M_PI_2;
	
	Ordre2.Type = AVANCE_Free;
	Ordre2.X = 225.0;
	Ordre2.Y = 330.0;
	Ordre2.Theta = -M_PI;
	
	Ordre3.Type = AVANCE_Free;
	Ordre3.X = 400.0;
	Ordre3.Y = 400.0;
	Ordre3.Theta = M_PI_2;
	
	Ordre4.Type = AVANCE;
	Ordre4.X = 0.0;
	Ordre4.Y = 0.0;
	Ordre4.Theta = 0;*/

//////////////////////////////////////////////////////////////////////////


#endif	
	
//////////////////////////////////////////////////////////////////////////
	while(1)
	{
		leds_on(LED1);
		
		if(goAsserv)
		{
			Init_codeurs(&codeur_d,&codeur_g); // initialisation codeuses
			
			nb_ordre_save();
			
			if (I2CNewOrderFlag) 
			{
				static int led=0;
				
				new_etat=(new_etat <(STACK_SIZE-1))? new_etat++:0;
								
				nb_ordre_save(); // mettre à jour le nombre d'ordre reçu (3 maxi) retour sur variable : nb_ordre
				
				//////////////////////////////////////////////////////////////////////////
				// convertion des ordes I2C en double ////////////////////////////////////
				Tab_ordre[new_etat]->Type = I2CNewOrder.Type;
				Tab_ordre[new_etat]->X = (double)I2CNewOrder.X/* / 10.0*/;
				Tab_ordre[new_etat]->Y = (double)I2CNewOrder.Y/* / 10.0*/;
				Tab_ordre[new_etat]->Theta = (double)I2CNewOrder.Theta/* / 10000.0*/;
				//////////////////////////////////////////////////////////////////////////
				
				
				if (led)
				{
					leds_off(LED3);
				}
				else
				{
					leds_on(LED3);
				}
								
				I2CNewOrderFlag = 0;
			}
			//////////////////////////////////////////////////////////////////////////
			// ODOMETRIE /////////////////////////////////////////////////////////////
			
			fct_odometrie();
			
			//////////////////////////////////////////////////////////////////////////
			// MACHINE A ETAT ////////////////////////////////////////////////////////	
			Actualisation_Ordre(STACK_SIZE);
					
			switch (Ordre_actuel.Type)
			{
// AVANCE
				case AVANCE :
				{
					position.Type=AVANCE;
					//////////////////////////////////////////////////////////////////////////
					// Trapèze vitesse avance ////////////////////////////////////////////////
					// calcule du déplassement anglulaire requis pour être à la bonne position
					Calcule_deplacement(AVANCE); // conversion position cartésienne en polaire erreur_U la distance et commande_Theta la distance	
					
					Vitesse_C_U = calculTrapez(AVANCE,Vitesse_C_U, 0/*V_Min*/, 5/*V_Max mm/10ms */, distance_restante, 2.0/*A_Desc mm/(10ms)²*/, 2.0/*A_Acc mm/(10ms)²*/, &positionnement_precis_U);
					Vitesse_C_T = calculTrapez(TOURNE, Vitesse_C_T, 0/*V_Min*/, 0.115/*V_Max rad/10ms*/, angle_restant, 3.0/*A_Desc rad/(10ms)² */, 3.0/*A_Acc mm/(10ms)² */, &positionnement_precis_T);
					
					
					/// Asservi PID
					// assignation aux variables des moteurs d'une nouvelle commande//////////
					
					// distance_Theta: vitesse en rad / 10 ms
					// distance_U: vitesse en mm / 10 ms
					
					PID_V_U= 6.6 * fct_PI(AVANCE,Ki_U,1.0 , Kp_U,Vitesse_C_U-distance_U,0.01);
					PID_V_T= 8.7 * fct_PI(TOURNE,Ki_T,1.0, Kp_T,Vitesse_C_T-distance_Theta,0.01);

					// assignation aux variables des moteurs d'une nouvelle commande//////////
					moteur_G = PID_V_U - PID_V_T ;
					moteur_D = PID_V_U + PID_V_T ;
					break;
				}

// >>> AVANCE_Free				
				case AVANCE_Free :
				{
					position.Type=AVANCE_Free;
					//////////////////////////////////////////////////////////////////////////
					// Trapèze vitesse avance ////////////////////////////////////////////////
					// calcule du déplassement anglulaire requis pour être à la bonne position
					
					V_Min=Calcul_ralentissement(Vitesse_C_U,30.0,position); // ATTETION la valeur V_Min en sortie est non signé
					
					Calcule_deplacement(AVANCE_Free); // conversion position cartésienne en polaire erreur_U la distance et commande_Theta la distance
					
					Vitesse_C_U = calculTrapez(AVANCE,Vitesse_C_U, 0/*V_Min*/, 5/*V_Max mm/10ms */, distance_restante, 2.0/*A_Desc mm/(10ms)² */, 2.0/*A_Acc mm/(10ms)² */, &positionnement_precis_U);
					Vitesse_C_T = calculTrapez(TOURNE, Vitesse_C_T, 0/*V_Min*/, 0.115/*V_Max rad/10ms */, angle_restant, 3.0/*A_Desc rad/(10ms)² */, 3.0/*A_Acc rad/(10ms)² */, &positionnement_precis_T);
					
					// Asservi PID
					// assignation aux variables des moteurs d'une nouvelle commande//////////
									
					PID_V_U=6.6 *fct_PI(AVANCE,Ki_U, 0 , Kp_U,Vitesse_C_U-distance_U,0.01);
					PID_V_T=8.7 *fct_PI(TOURNE,Ki_T,1, Kp_T,Vitesse_C_T-distance_Theta,0.01);
					
					moteur_G =PID_V_U - PID_V_T ;
					moteur_D =PID_V_U + PID_V_T ;
					
					break;
				}
				
// TOURNE
				case TOURNE :
				{
					position.Type=TOURNE;
					//////////////////////////////////////////////////////////////////////////
					// Trapèze vitesse angulaire /////////////////////////////////////////////
					
					Calcule_deplacement(TOURNE); // conversion position cartésienne en polaire erreur_U la distance et commande_Theta la distance
					
					// Vmax: mm / 10ms
					// angle_restant: angle en rad
					
					Vitesse_C_U = calculTrapez(AVANCE,Vitesse_C_U, 0.0/*V_Min*/, 15.0/*V_Max mm/10ms */, distance_restante, 2.0/*A_Desc mm/(10ms)² */, 2.0/*A_Acc mm/(10ms)² */, &positionnement_precis_U);
					Vitesse_C_T = calculTrapez(TOURNE, Vitesse_C_T, 0.0/*V_Min*/, 0.115/*V_Max rad/10ms */, angle_restant, 3.5/*A_Desc rad/(10ms)² */, 2.5/*A_Acc rad/(10ms)² */, &positionnement_precis_T);
					
					
					/// Asservi PID
					// assignation aux variables des moteurs d'une nouvelle commande//////////
					
					// distance_Theta: vitesse en rad / 10 ms
					// distance_U: vitesse en mm / 10 ms
					
					
					// facteur calculé 6.6 et 8.7
					PID_V_U= 6.6 *fct_PI(AVANCE,Ki_U,1 , Kp_U,Vitesse_C_U-distance_U,0.01);
					PID_V_T= 8.7 *fct_PI(TOURNE,Ki_T,1, Kp_T,Vitesse_C_T-distance_Theta,0.01);
					
					moteur_G = PID_V_U - PID_V_T ;
					moteur_D = PID_V_U + PID_V_T ;					
					break;
				}
				
// >>> STOP					
				//Par I2C	
				case STOP :
				{
					position.Type=STOP;
					// Initialisation
					moteur_G = 0 ;
					moteur_D = 0 ;
					Vitesse_C_T=0;
					Vitesse_C_U=0;
					distance_restante=0;
					angle_restant=0;
						
					// passage à l'orde suivant si un nouveau à été envoyé
					positionnement_precis_U=1;
					positionnement_precis_T=1;
					break;
				}	
			}

			//////////////////////////////////////////////////////////////////////////
			// Bloquage
			//detecBugBloquage();
			//////////////////////////////////////////////////////////////////////////
	
#if TEST_CODEUSES	
			tot_codeur_D=tot_codeur_D+Sens_codeur_D*(codeur_d - 10000);
			tot_codeur_G=tot_codeur_G+Sens_codeur_G*(codeur_g - 10000);
			
			
			if(tot_codeur_D>=20000)
			{
				tot_codeur_D-=20000;
				tot_codeur_D_2++;
			}
			if(tot_codeur_D<=-20000)
			{
				tot_codeur_D+=20000;
				tot_codeur_D_2--;
			}
			
		
			if(tot_codeur_G>=20000)
			{
				tot_codeur_G-=20000;
				tot_codeur_G_2++;
			}
			if(tot_codeur_G<=-20000)
			{
				tot_codeur_G+=20000;

				tot_codeur_G_2--;
			}
#endif					
			
			goAsserv = 0; // fin de mise à jour de l'Asservissement
		} // end if goAsserv	
		
//MOTEUR
		//////////////////////////////////////////////////////////////////////////
		///////////////Mise à jour des registres des pwm commandant les moteurs///
		// assignation aux variables des moteurs d'une nouvelle commande /////////
		pwm_set(cablage_mot_D,Sens_mot_D*moteur_D); // attention moteur gauche inversé
		pwm_set(cablage_mot_G,Sens_mot_G*moteur_G);// modifié pour le petit robot
	
		//////////////////////////////////////////////////////////////////////////
		fct_Ordre_suivant(&positionnement_precis_U,&positionnement_precis_T,STACK_SIZE,STOP);
		//////////////////////////////////////////////////////////////////////////
	} // end while(1)
	
} // end fct main


//////////////////////////////////////////////////////////////////////////
// Que se passe-t-il quand on désactive les interruptions au moment de cette interruption
// Qu'elle sera l'impacte sur le calcule de la vitesse de déplacement (constente de temps plus précisément
//  DANS config.c :
//		- Valeur de Comparé : 0x9C40=40000(dec) (TC_SetCompareA( &TCD1, 0x9C40 ); )
//		- Clock : 32Mhz (CLKSYS_Main_ClockSource_Select( CLK_SCLKSEL_RC32M_gc );)
//		- Sensibilité du Timer : Clock/8=4Mhz (TC1_ConfigClockSource( &TCD1, TC_CLKSEL_DIV8_gc );)
//	=>(1/4Mhz)*40000=10ms
ISR(TCD1_CCA_vect) // Interruption sur Timer permettant un rebouclage temporisé
{
	TC_Restart( &TCD1 ); // redémarrer le timer
	TC_SetCount(&TCD1,0); // Mettre le Compter A du Timer/Comparateur D1 à 0

	goAsserv = 1; // Demande de mise a jour asservissement
}


