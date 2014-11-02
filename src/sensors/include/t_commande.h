typedef enum
		{
			CMD_DUMMY = '0',
			CMD_SND_COMP = 'c',
			CMD_SND_ETAT = 'e',
			CMD_SND_BATT = 'b',
			CMD_SND_PWM = 'w',
			CMD_SND_POSVIT = 'v',
			CMD_SND_PDES = 'd',
			CMD_SND_SONAR = 's',
			CMD_SND_OPT = 'o',
			CMD_REC_ETAT = 'g',	//Goto state
			CMD_REC_MOTCMD = 'f',	//Out Control (Forces)
			CMD_REC_MAN = 'm',	//Move
			CMD_REC_PDES =  'a',	//Automatic
			CMD_REC_POS = 'l',	//Location
			CMD_INCONNUE = '?'
		}
		T_COMMANDE;

typedef enum
		{
			NONE = 				0,
			TOUCHE_PILOTAGE_CLAVIER_ON =	'm',		//27			// touche clavier pour entrer du mode pilotage manuel
			TOUCHE_PILOTAGE_CLAVIER_OFF =	'x',		//27			// touche clavier pour sortir du mode pilotage manuel
			TOUCHE_AVANT =			'*',
			TOUCHE_ARRIERE =		'6',
			TOUCHE_GAUCHE =			'8',
			TOUCHE_DROITE =			'-',
			TOUCHE_HAUT =			'1',
			TOUCHE_BAS =			'0',
			TOUCHE_ROTATION_GAUCHE =	'/',
			TOUCHE_ROTATION_DROITE =	'+',
			TOUCHE_ROTATION_YP =		'2',
			TOUCHE_ROTATION_YN =		'3',
			TOUCHE_ROTATION_XP =		'7',
			TOUCHE_ROTATION_XN =		'4',
			//TOUCHE_ARRET_X =		'1',
			//TOUCHE_ARRET_Y =		'3',
			TOUCHE_ARRET_XY =		'9',
			TOUCHE_ARRET_XYZ =		'.',
			TOUCHE_ARRET_Z =		'5'
		}
		T_TOUCHE;
