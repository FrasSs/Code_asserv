
#include "../include/Ordres.h"


void Actualisation_Ordre(int stack)
{
	static int etat_suiv=0;
	Ordre_actuel.Type=Tab_ordre[etat]->Type;
	Ordre_actuel.X=Tab_ordre[etat]->X;
	Ordre_actuel.Y=Tab_ordre[etat]->Y;
	Ordre_actuel.Theta=Tab_ordre[etat]->Theta;
	
	etat_suiv = (etat<(stack-1)) ? etat+1 : 0;
	Ordre_suivant.Type=Tab_ordre[etat_suiv]->Type;
	Ordre_suivant.X=Tab_ordre[etat_suiv]->X;
	Ordre_suivant.Y=Tab_ordre[etat_suiv]->Y;
	Ordre_suivant.Theta=Tab_ordre[etat_suiv]->Theta;
}


void fct_Ordre_suivant(int *cond1,int *cond2,int stack,int stop)
{
	if ((*cond1==1)&&(*cond2==1)) // si fin d'état
	{		
		if ((nb_ordre-1)>0)
		{// incrément jusqu'à attendre new_etat
			
			Tab_ordre[etat]->Type=stop;
			Tab_ordre[etat]->X=0;
			Tab_ordre[etat]->Y=0;
			Tab_ordre[etat]->Theta=0;
			*cond1=0;
			*cond2=0;
			etat=(etat<(stack-1)) ? etat+1 : 0;
		}
		
		nb_ordre_save(); // mettre à jour le nombre d'ordre reçu (3 maxi) retour sur variable : nb_ordre
	}
}