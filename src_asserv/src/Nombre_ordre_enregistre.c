/*
 * CFile1.c
 *
 * Created: 27/09/2012 19:33:14
 *  Author : Léon
 *	Corrigé : François
 */ 

#include <stdint.h>
#include "../include/Nombre_ordre_enregistre.h"
extern uint8_t new_etat;
extern uint8_t etat;
extern int nb_ordre;
extern Commande * Tab_ordre[];

void nb_ordre_save(void)
{
	int i,j;
	
	if (etat!=new_etat)
	{
		for(i=0,j=etat;j!=new_etat;i++)
		{
			j=((j+1)<STACK_SIZE) ? j+1 : 0;
		}
		
		nb_ordre = i;
	}
	else
	{
		if (nb_ordre)
		{
			nb_ordre=4;
		}
		else
		{
			nb_ordre=0;
		}
	}
}