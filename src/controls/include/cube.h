//#define MC160
#define MC225

#ifdef MC160
#define CUBE_LENGTH		1		// Cube length (m)

#define MIN_CUBE_MASS		0		// Minimal cube mass (kg)
#define CUBE_MASS		1		// Estimated cube mass (kg)
#define MAX_CUBE_MASS		4		// Maximal cube mass (kg)

#define MIN_CUBE_INERTIA	0		// Min Cube inertia (kg*m^2)
#define CUBE_INERTIA		2		// Cube inertia (kg*m^2)
#define MAX_CUBE_INERTIA	8		// Max Cube inertia (kg*m^2)

#define MIN_CUBE_KAIR1		0		// Minimal Lateral air resistance
#define CUBE_KAIR1		2.4		// Estimated Lateral air resistance
#define MAX_CUBE_KAIR1		4		// Maximal Lateral air resistance

#define MIN_CUBE_KAIR2		0		// Minimal Rotational air resistance
#define CUBE_KAIR2		2.4		// Rotational air resistance
#define MAX_CUBE_KAIR2		4		// Maximal Rotational air resistance
#define MOTOR_GAIN		350		// uster selon la turbine utilis tel que 1 Fx = 1N. (Tests avec capteurs de tension)

#else
#define CUBE_LENGTH		2.25		// Cube length (m)

#define MIN_CUBE_MASS		0		// Minimal cube mass (kg)
#define CUBE_MASS		11		// Estimated cube mass (kg)
#define MAX_CUBE_MASS		13		// Maximal cube mass (kg)

#define MIN_CUBE_INERTIA	0		// Min Cube inertia (kg*m^2)
#define CUBE_INERTIA		14		// Cube inertia (kg*m^2)
#define MAX_CUBE_INERTIA	20		// Max Cube inertia (kg*m^2)  //ATTENTION TROP GRAND??? = agressif

#define MIN_CUBE_KAIR1		0		// Minimal Lateral air resistance
#define CUBE_KAIR1		9.5		// Estimated Lateral air resistance (prop. < la surf.)
#define MAX_CUBE_KAIR1		16		// Maximal Lateral air resistance

#define MIN_CUBE_KAIR2		0		// Minimal Rotational air resistance
#define CUBE_KAIR2		9.5		// Rotational air resistance
#define MAX_CUBE_KAIR2		16		// Maximal Rotational air resistance
#define MOTOR_GAIN		255		// uster selon la turbine utilis el que 1 Fx = 1N. (Tests avec capteurs de tension)
#endif

#define CFG_6DDL		0


// distances
#define	DISTANCE_MAX				600		// cm (limite sonar) (Temporairement r�duit de 600 � 200cm pour le centre des sciences)
#define	DISTANCE_INDEFINIE			(short)0xFFFF
#define	DISTANCE_OBSTACLE			350 //60MW		// cm
#define	DISTANCE_SECURITE			60 //110MW		// cm
#define	DISTANCE_SECURITEZ			30		// cm
#define	DISTANCE_VISITEUR			200		// cm ... le cube ne se stabilisait pas dans l'entonnoir a 200cm
#define DISTANCE_INTERACTION			50		// cm Distance qui va declanger l'interaction sur des capteurs en haut >TEMP 5cm pour eviter contact
#define DISTANCE_INTERACTION_BAS		20		// cm Distance qui va declanger l'interaction sur des capteurs en bas. Doit �tre > DISTANCE_SECURITE  >TEMP 5cm pour eviter contact

// luminosite
#define	LUMINOSITE_MAX				255		// 
#define	LUMINOSITE_STABILISATION		180		// 
#define	LUMINOSITE_OBJET			50		// unites
#define LUMINOSITE_INTERACTION			0 //80		//0 Pas dinteraction, negatif attire, positif effraye
#define LUMINOSITE_DISTANCE			150		//DISTANCE = LUMINOSITE/LUMINOSITE_DISTANCE

// orientation
#define	ANGLE_INDEFINI				-360.1F		// degres
#define	TEMPERATURE_INDEFINIE			100.1F		// degres

// Environnement SICK
#define MAX_X					10	//m
#define MIN_X					0.2	//m
#define MAX_Y					5	//m
#define MIN_Y					-5	//m
#define MAX_Z					3	//m
#define MIN_Z					0	//m
#define MAX_TZ					3.14	//rad
#define MIN_TZ					0	//rad