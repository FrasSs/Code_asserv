﻿/*
 * CFile1.c
 *
 * Created: 27/09/2012 19:33:14
 *  Author : Léon
 *	Corrigé : François
 */ 

#include <stdint.h>
#include "../include/struct_Commande.h"
extern int new_etat;
extern uint8_t etat;
extern int nb_ordre;
extern Commande * Tab_ordre[];

void nb_ordre_save(void)
{

	int i=0,j=etat;
	int stack;
	
	for(stack=0;Tab_ordre[stack]->Type!=0;stack++); // compt stack
	
	for(i=0,j=etat;j!=new_etat;i++)
	{
		j=(j<stack) ? j+1 : 0;
	}
	nb_ordre = i;
}