#ifndef DETECT_BB
#define DETECT_BB

#include "config.h"
#include "STOP_Force.h"

extern Commande Ordre_actuel;
extern double moteur_G;
extern double moteur_D;

void detecBugBloquage(void);
void acknowledge_bloquage(void);

#endif
