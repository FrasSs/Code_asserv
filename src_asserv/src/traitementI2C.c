
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
		case AVANCE_TOURNE :
		{
			for (int i=0; i<size; i++) {
				*(i2c_envoie.X+i)=msg[i+1];
			}
			i2c_packetToI2CNewOrder();
			break;
		}
		case POSITION_R :
		{
			//////////////////////////////////////////////////////////////////////////
			// Write i2C

			positionToI2c_packet();

			for (int i=0; i<12; i++) {
				buffer_OUT[i]=*(i2c_envoie.X+i);
			}
			i2cWrite(buffer_OUT, 12);
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
			
		case POSITION_W:
		{
			temp = ((long)((unsigned char)msg[4]))<<24;
			temp += ((long)((unsigned char)msg[3]))<<16;
			temp += ((long)((unsigned char)msg[2]))<<8;
			temp += ((long)((unsigned char)msg[1]));
			position.X=((double)temp / (double)10);

			//Reception de Y en dixieme de mm (1 = 0.0001 metre)
			temp = ((long)((unsigned char)msg[8]))<<24;
			temp += ((long)((unsigned char)msg[7]))<<16;
			temp += ((long)((unsigned char)msg[6]))<<8;
			temp += ((long)((unsigned char)msg[5]));
			position.Y=((double)temp / (double)10);

			//Reception de theta en dixieme de miliRadians(1 = 0.0001 radians)
			temp = ((long)((unsigned char)msg[12]))<<24;
			temp += ((long)((unsigned char)msg[11]))<<16;
			temp += ((long)((unsigned char)msg[10]))<<8;
			temp += ((long)((unsigned char)msg[9]));
			position.Theta=((double)temp / (double)10000);
				
			for(eff_ordre=0;Tab_ordre[eff_ordre+1]->Type==0;eff_ordre++) // effassement de tous les ordres
			{
				Tab_ordre[eff_ordre]->Type=STOP;
				Tab_ordre[eff_ordre]->X=0;
				Tab_ordre[eff_ordre]->Y=0;
				Tab_ordre[eff_ordre]->Theta=0;
			}
			I2CNewOrderFlag = 1;
			break;
		}
			
			
		case AVANCE:
		{
			temp = ((long)((unsigned char)msg[4]))<<24;
			temp += ((long)((unsigned char)msg[3]))<<16;
			temp += ((long)((unsigned char)msg[2]))<<8;
			temp += ((long)((unsigned char)msg[1]));
			I2CNewOrder.X=((double)temp / (double)10);

			//Reception de Y en dixieme de mm (1 = 0.0001 metre)
			temp = ((long)((unsigned char)msg[8]))<<24;
			temp += ((long)((unsigned char)msg[7]))<<16;
			temp += ((long)((unsigned char)msg[6]))<<8;
			temp += ((long)((unsigned char)msg[5]));
			I2CNewOrder.Y=((double)temp / (double)10);

			//Reception de theta en dixieme de miliRadians(1 = 0.0001 radians)
			temp = ((long)((unsigned char)msg[12]))<<24;
			temp += ((long)((unsigned char)msg[11]))<<16;
			temp += ((long)((unsigned char)msg[10]))<<8;
			temp += ((long)((unsigned char)msg[9]));
			I2CNewOrder.Theta=((double)temp / (double)10000.0);
				
			I2CNewOrderFlag = 1;
			break;
		}
	}
}

 int positionToI2c_packet() {
   i2c_envoie.Type=position.Type;
   uint8_t signe[3]={0};
   uint32_t masque=0xFF;
   double monX=position.X;
   double monY=position.Y;
   double monTheta=position.Theta;
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
      i2c_envoie.X[i]=(uint8_t)((masque)&(((uint32_t)(monX*10))>>(i*8)));
      i2c_envoie.Y[i]=(uint8_t)((masque)&(((uint32_t)(monY*10))>>(i*8)));
      i2c_envoie.Theta[i]=(uint8_t)((masque)&(((uint32_t)(monTheta*10000))>>(i*8)));
    }
    i2c_envoie.X[3]|=signe[0]<<7;
    i2c_envoie.Y[3]|=signe[1]<<7;
    i2c_envoie.Theta[3]|=signe[2]<<7;
    return 0;
  }

int i2c_packetToI2CNewOrder() {
  I2CNewOrder.Type=i2c_envoie.Type;
  double monX;
  double monY;
  double monTheta;
  int32_t temp[3]={0};
  double signe[3]={1};
  for (int i=0; i<4; i++) {
    temp[0]|=((uint32_t)(i2c_envoie.X[i])) << (i*8);
    temp[1]|=((uint32_t)(i2c_envoie.Y[i])) << (i*8);
    temp[2]|=((uint32_t)(i2c_envoie.Theta[i])) << (i*8);
  }
  uint32_t olol=0;
  for (int i=0; i<3; i++) {
    if (temp[i]>>(8*4-1)) {
      signe[i]=-1;
      olol=1;
      olol=olol<<(8*4-1);
      temp[i]&=~olol;
    }
  }
  I2CNewOrder.X=signe[0]*((double)(temp[0]))/10.0;
  I2CNewOrder.Y=signe[1]*((double)(temp[1]))/10.0;
  I2CNewOrder.Theta=signe[2]*((double)(temp[2]))/10000.0;
  return 0;
}


