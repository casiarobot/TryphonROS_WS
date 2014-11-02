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
#define CUBE_MASS			3.5		// Estimated cube mass (kg)
#define MAX_CUBE_MASS		5		// Maximal cube mass (kg)

#define MIN_CUBE_INERTIA	0		// Min Cube inertia (kg*m^2)
#define CUBE_INERTIA		16		// Cube inertia (kg*m^2)
#define MAX_CUBE_INERTIA	30		// Max Cube inertia (kg*m^2)  //ATTENTION TROP GRAND??? = agressif

#define MIN_CUBE_KAIR1		0		// Minimal Lateral air resistance
#define CUBE_KAIR1			9.5		// Estimated Lateral air resistance (prop. < la surf.)
#define MAX_CUBE_KAIR1		16		// Maximal Lateral air resistance

#define MIN_CUBE_KAIR2		0		// Minimal Rotational air resistance
#define CUBE_KAIR2			9.5		// Rotational air resistance
#define MAX_CUBE_KAIR2		16		// Maximal Rotational air resistance
#define MOTOR_GAIN		255		// uster selon la turbine utilis el que 1 Fx = 1N. (Tests avec capteurs de tension)
#endif

#define CFG_6DDL		0
