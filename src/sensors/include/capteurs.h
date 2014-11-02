/***************************************************************
\ Adresses de sonars
\ Definition de leur registres sonar
***************************************************************/
// SRF08 (gros modele avec senseur de lumiere)
#define	SONAR_AVANT_GAUCHE		0xE0
#define	SONAR_AVANT_DROITE		0xE2
#define	SONAR_DROITE_AVANT		0xE4
#define	SONAR_DROITE_ARRIERE		0xE6
#define	SONAR_ARRIERE_DROITE		0xE8
#define	SONAR_ARRIERE_GAUCHE		0xEA
#define	SONAR_GAUCHE_ARRIERE		0xEC
#define	SONAR_GAUCHE_AVANT		0xEE
// SRF10 (petit modele sans senseur de lumiere)
#define	SONAR_AVANT_HAUT		0xF0
#define	SONAR_DROITE_HAUT		0xF2
#define	SONAR_ARRIERE_HAUT		0xF4
#define	SONAR_GAUCHE_HAUT		0xF6
#define	SONAR_HAUT1			0xF8	// ajouts vers le bas a l avant - FNC08
#define	SONAR_HAUT2			0xFA	// ajouts vers le bas a droite - FNC08
#define	SONAR_BAS1			0xFC
#define	SONAR_BAS2			0xFE
// registres en ecriture
#define	SONAR_COMMAND			0x00	// commande
#define	SONAR_GAIN			0x01	// gain
#define	SONAR_RANGE			0x02	// delai autorise pour l'acquisition
// registres en lecture
#define	SONAR_SOFTWARE_REVISION		0x00	// version logiciel
#define	SONAR_LIGHT			0x01	// luminosite
#define	SONAR_FIRST_ECHO_HIGH		0x02	// octet de poids fort de la distance
#define	SONAR_FIRST_ECHO_LOW		0x03	// octet de poids faible de la distance
// commandes
#define	SONAR_RANGING_CM		0x51	// demande d'acquisition en CM
#define	SONAR_GAIN_DEFAULT		10	// gain par defaut
#define	SONAR_RANGE_DEFAULT		140	// range par defaut
// Thread periode - 100
#define	SONAR_PERIODE			100	// periode d'echantillonnage en MS
#define DISTANCE_MAX                    600
/***************************************************************
\ Adresses des indicateurs lumineux (PCA)
\ Definition de leur registres
***************************************************************/
// adresse I2C
#define MIN_IND				0
#define IND_LUMINEUX_0			0x12//0xCE//C0
#define IND_LUMINEUX_2			0x14//0xC2
#define IND_LUMINEUX_4			0x16//0xC4
#define IND_LUMINEUX_6			0x18//0xC6
#define IND_LUMINEUX_8			0xC8
#define IND_LUMINEUX_A			0xCA
#define IND_LUMINEUX_C			0xCC
#define IND_LUMINEUX_E			0xD2
#define MAX_IND				8
//Registres en ecriture
#define PSC0				1
#define PWM0				2
#define PSC1				3
#define PWM1				4
#define LEDSEL				5
#define goRGB				110
#define fadeRGB				99
#define stopS				111

/***************************************************************
\ Adresses des Niveaux de batteries (LTC)
\ Definition de leur registres
***************************************************************/
// adresse I2C
#define MIN_BATT			0
#define IND_BATT_1			0xCE
#define IND_BATT_2			0xDC
#define MAX_BATT			2
// registres en lecture
#define	BATT_SENSEA			0x00
#define	BATT_SENSEB			0x01
#define	BATT_VINC			0x02
#define	BATT_VIND			0x03
//ADC
// the valid range is 0 - 2500, so this will work for an error
#define ADC_READ_ERROR -100000

#define ADC2 0
#define ADC3 1
#define ADC4 2
#define ADC5 3
#define ADC6 4
#define ADC7 5
#define NUM_ADC 6

/***************************************************************
\ Adresse de l'inclinomètre (dernière utilisation en 2006)
\ Définition de ses registres
***************************************************************/
// adresse I2C  --Dernière utilisation à 0xC2
#define	INCLINOMETRE			0xD6
// valeurs
#define	UNKNOWN        			((signed short)0xFFFE)
#define	NOT_READY      			((signed short)0xFFFF)
// registres en ecriture
#define	INCLINOMETRE_INIT		0x01
#define	INCLINOMETRE_UPDATE		0x03
// registres en lecture
#define	INCLINOMETRE_READ_PITCH		0x04
#define	INCLINOMETRE_READ_ROLL		0x05
#define	INCLINOMETRE_READ_TEMPERATURE	0x06

/***************************************************************
\ Adresse de la boussole
\ Définition de ses registres
***************************************************************/
// adresse I2C
#define	BOUSSOLE			0xC0
//#define	BOUSSOLE			0x32
// registres en lecture
#define BOUSSOLE_SOFTWARE_REVISION	0x00
#define BOUSSOLE_BEARING_WORD_HIGH	0x02
#define BOUSSOLE_BEARING_WORD_LOW	0x03
#define HMC6343_HEADING			0x50		
// Thread periode
#define	BOU_PERIODE		100	// periode d'echantillonnage en MS


/***************************************************************
\ Adresse de l'altimetre
\ Definition de ses registres
***************************************************************/
// adresse I2C
#define	ALTI_ADD			0x22
// registres en ecriture
#define ALTI_CTRL_OP			0x03
#define ALTI_OP_HR			0x0A
#define ALTI_OP_HS			0x09
#define ALTI_OP_TRIG			0x0C
#define ALTI_OP_STOP			0x00
// registres en lecture
#define ALTI_PMSB_REG			0x7F
#define ALTI_PLSB_REG			0x80
#define ALTI_TEMP_REG			0x81
#define ALTI_STATUS_REG			0x07
//Constante (AIR=288.15/0.0065)
#define ALTI_AIR_CST			44330
#define ALTI_P0				101325


/***************************************************************
\ Adresse de l'accelerometre
\ Definition de ses registres
***************************************************************/
// adresse I2C
#define	ACCEL_ADD		0x3a
// registres en .criture
#define ACCEL_OFFX		0x16
#define ACCEL_OFFY		0x17
#define ACCEL_OFFZ		0x18
#define ACCEL_GAINX		0x19
#define ACCEL_GAINY		0x1A
#define ACCEL_GAINZ		0x1B
#define ACCEL_CTRL_REG1		0x20
#define ACCEL_CTRL_REG2		0x21
#define ACCEL_CTRL_REG3		0x22
//valeurs
//---All device on; decimation factor=40Hz(min)
#define ACCEL_REG1_DEF		199
//Self-testing enable
#define ACCEL_REG1_ST		207
//-------Big Endian; 2g; 16bit
//#define ACCEL_REG2_DEF		33
//-------Little Endian; 2g; 12bit
#define ACCEL_REG2_DEF		32
//-------High pass filter ON - 512
#define ACCEL_REG3_DEF		96
// registres en lecture
#define ACCEL_XWORD_HIGH	0x28
#define ACCEL_XWORD_LOW		0x29
#define ACCEL_YWORD_HIGH	0x2A
#define ACCEL_YWORD_LOW		0x2B
#define ACCEL_ZWORD_HIGH	0x2C
#define ACCEL_ZWORD_LOW		0x2D
// Thread periode
#define	ACCEL_PERIODE		250	// periode d'echantillonnage en MS

