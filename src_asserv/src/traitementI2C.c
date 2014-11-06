
//*****************************************************************************
// PIC_Asser_Moteur
// traitementI2C.h
// dsPIC 30F
// V.MATOUILLOT 26/01/08
//*****************************************************************************
#include <math.h>
#include "../include/traitementI2C.h"
#include "../hardware/i2c.h"
#include <avr/io.h>
#include "../hardware/leds.h"
#include "../include/struct_Commande.h"


#include "../hardware/i2c.h"
//== PUBLIC VARIABLES =========================================================
uint8_t I2CNewOrderFlag = 0;
uint8_t I2CNewOrderAkwnoledged = 1;
extern Commande I2CNewOrder;
extern i2c_packet i2c_envoie;

extern Commande position;
extern int nb_ordre;

// BUFFER ( 20 octets ) : Reception
unsigned char msg[20];
unsigned char* pmsg = & msg[0];
unsigned char msg_size;

// BUFFER ( 20 octets ) : Emission
unsigned char buffer_OUT[20];
unsigned char buffer_OUT_INDEX;
int buffer_OUT_size;

unsigned char ERREURI2C=0;
unsigned int  seuil_blockage=0;

void i2cCallBack(uint8_t cmd, uint8_t dataRx[], uint8_t size)
{
	static int led=0;
	
	traitementI2C(--dataRx, ++size);
	
	if (led)
	{
		leds_off(LED2);
	} 
	else
	{
		leds_on(LED2);
	}
	
	
}

void traitementI2C(unsigned char * msg,unsigned char size)
{
	int eff_ordre=0;
	long temp;
		
	//////////////////////////////////////////////////////////////////////////
	// Read I2C
				
	temp = ((long)((unsigned char)msg[0]));
	i2c_envoie.Type=(uint8_t)temp;
	switch (i2c_envoie.Type)
	{
		case AVANCE :
		{
			for (int i=0; i<size; i++) {
				*(i2c_envoie.X+i)=msg[i+1];
			}
			i2c_packetToCommande(&i2c_envoie, &I2CNewOrder);
			I2CNewOrderFlag = 1;
			break;
		}
		
		case TOURNE:
		{
			for (int i=0; i<size; i++) {
				*(i2c_envoie.X+i)=msg[i+1];
			}
			
			i2c_packetToCommande(&i2c_envoie, &I2CNewOrder);
			I2CNewOrderFlag = 1;
			break;
		}
		
		case POSITION_R :
		{
			//////////////////////////////////////////////////////////////////////////
			// Write i2C

			commandeToI2c_packet(&position, &i2c_envoie);
			i2c_envoie.Type=Ordre_actuel.Type;
			for (int i=0; i<13; i++) {
				buffer_OUT[i]=*((i2c_envoie.X-1)+i);
			}
			
			i2cWrite(buffer_OUT, 13);
			I2CNewOrderFlag = 0;
			break;
		}
		
		case POSITION_W:
		{
			for (int i=0; i<size; i++) {
				*(i2c_envoie.X+i)=msg[i+1];
			}
			i2c_packetToCommande(&i2c_envoie, &position);
			I2CNewOrderFlag = 0;
			break;
		}
		
		case STATUS_ROB :
		{
			if (!Mouvement_robot())
			{
				buffer_OUT[0] = Ordre_actuel.Type;
			}
			else
			{
				buffer_OUT[0] = STOP;
			}
				
			i2cWrite(buffer_OUT, 1);
			I2CNewOrderFlag = 0;
			break;
		}
			
		case STOP_F:
		{
			fct_STOP_F(STOP);
			I2CNewOrder.Type=STOP;
			I2CNewOrderFlag = 1;
			break;
		}
	}
}

void commandeToI2c_packet(const Commande *myCommande, i2c_packet *myI2c_packet) {
	myI2c_packet->Type=myCommande->Type;
	uint8_t signe[3]={0};
	uint32_t masque=0xFF;
	double monX=myCommande->X;
	double monY=myCommande->Y;
	double monTheta=myCommande->Theta;
	if (monX<0) {
		signe[0]=1;
		monX*=-1;
	}
	if (monY<0) {
		signe[1]=1;
		monY*=-1;
	}
	if (monTheta<0) {
		signe[2]=1;
		monTheta*=-1;
	}
	for (int i=0; i<4; i++) {
		if (i==3) masque=0x7F;
		myI2c_packet->X[i]=(uint8_t)((masque)&(((uint32_t)(monX*10))>>(i*8)));
		myI2c_packet->Y[i]=(uint8_t)((masque)&(((uint32_t)(monY*10))>>(i*8)));
		myI2c_packet->Theta[i]=(uint8_t)((masque)&(((uint32_t)(monTheta*10000))>>(i*8)));
	}
	myI2c_packet->X[3]|=signe[0]<<7;
	myI2c_packet->Y[3]|=signe[1]<<7;
	myI2c_packet->Theta[3]|=signe[2]<<7;
}

void i2c_packetToCommande(const i2c_packet *myI2c_packet, Commande *myCommande) {
	myCommande->Type=myI2c_packet->Type;
	int32_t temp[3]={0, 0, 0};
	int32_t temp_mask=0;
	double signe[3]={1, 1, 1};
	for (int i=0; i<4; i++) {
		temp[0]|=((uint32_t)(myI2c_packet->X[i])) << (i*8);
		temp[1]|=((uint32_t)(myI2c_packet->Y[i])) << (i*8);
		temp[2]|=((uint32_t)(myI2c_packet->Theta[i])) << (i*8);
	}
	for (int i=0; i<3; i++) {
		if (temp[i]>>(8*4-1)) {
			signe[i]=-1;
			temp_mask=1;
			temp_mask=temp_mask<<(8*4-1);
			temp_mask=~temp_mask;
			temp[i]&=temp_mask;
		}
	}
	myCommande->X=signe[0]*((double)(temp[0]))/10.0;
	myCommande->Y=signe[1]*((double)(temp[1]))/10.0;
	myCommande->Theta=signe[2]*((double)(temp[2]))/10000.0;
}
