#include "common.h"
#include "GCode.h"



//*******************************M80***************************************
//M80
void addM80RegulationAPI()
{
    #if SWING_ARM_MACHINE == 1
	addMCommadRegulation(80, IO_PRE_ACTION, cMk, 1, NULL, NULL, NULL, NULL, AssignM80);
#endif
}

//?M80
int8 AssignM80(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit)
{
	if (pSAssignConfig == NULL || pGCodeBufUnit == NULL)
	{
		return -1;
	}

	setIO(M80, pGCodeBufUnit->ucIOCommandArray, IO_MAXGROUP_IN_SINGLE_LINE);
	return 1;
}

//*******************************M81***************************************
//M81
void addM81RegulationAPI()
{
#if SWING_ARM_MACHINE == 1
	addMCommadRegulation(81, IO_PRE_ACTION, cMk, 1, NULL, NULL, NULL, NULL, AssignM81);
#endif
}

//?M81
int8 AssignM81(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit)
{
	if (pSAssignConfig == NULL || pGCodeBufUnit == NULL)
	{
		return -1;
	}

	setIO(M81, pGCodeBufUnit->ucIOCommandArray, IO_MAXGROUP_IN_SINGLE_LINE);
	return 1;
}

//*******************************M82***************************************
//M82
void addM82RegulationAPI()
{
#if SWING_ARM_MACHINE == 1
	addMCommadRegulation(82, IO_PRE_ACTION, cMk, 1, NULL, NULL, NULL, NULL, AssignM82);
#endif
}

//?M82
int8 AssignM82(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit)
{
	if (pSAssignConfig == NULL || pGCodeBufUnit == NULL)
	{
		return -1;
	}

	setIO(M82, pGCodeBufUnit->ucIOCommandArray, IO_MAXGROUP_IN_SINGLE_LINE);
	return 1;
}

//*******************************M83***************************************
//M83
void addM83RegulationAPI()
{
#if SWING_ARM_MACHINE == 1
	addMCommadRegulation(83, IO_PRE_ACTION, cMk, 1, NULL, NULL, NULL, NULL, AssignM83);
#endif
}

//?M83
int8 AssignM83(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit)
{
	if (pSAssignConfig == NULL || pGCodeBufUnit == NULL)
	{
		return -1;
	}

	setIO(M83, pGCodeBufUnit->ucIOCommandArray, IO_MAXGROUP_IN_SINGLE_LINE);
	return 1;
}


//*******************************M84***************************************
//M84
void addM84RegulationAPI()
{
#if SWING_ARM_MACHINE == 1
	addMCommadRegulation(84, IO_PRE_ACTION, cMk, 1, NULL, NULL, NULL, NULL, AssignM84);
#endif
}

//?M84
int8 AssignM84(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit)
{
	if (pSAssignConfig == NULL || pGCodeBufUnit == NULL)
	{
		return -1;
	}

	setIO(M84, pGCodeBufUnit->ucIOCommandArray, IO_MAXGROUP_IN_SINGLE_LINE);

}


//*******************************M85***************************************
//M85
void addM85RegulationAPI()
{
#if SWING_ARM_MACHINE == 1
	addMCommadRegulation(85, IO_PRE_ACTION, cMk, 1, NULL, NULL, NULL, NULL, AssignM85);
#endif
}

//?M85
int8 AssignM85(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit)
{
	if (pSAssignConfig == NULL || pGCodeBufUnit == NULL)
	{
		return -1;
	}

	setIO(M85, pGCodeBufUnit->ucIOCommandArray, IO_MAXGROUP_IN_SINGLE_LINE);

}

//*******************************M86***************************************
//M86
void addM86RegulationAPI()
{
#if SWING_ARM_MACHINE == 1
	addMCommadRegulation(86, IO_PRE_ACTION, cMk, 1, NULL, NULL, NULL, NULL, AssignM86);
#endif
}

//?M86
int8 AssignM86(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit)
{
	if (pSAssignConfig == NULL || pGCodeBufUnit == NULL)
	{
		return -1;
	}

	setIO(M86, pGCodeBufUnit->ucIOCommandArray, IO_MAXGROUP_IN_SINGLE_LINE);

}

//*******************************M87***************************************
//M87
void addM87RegulationAPI()
{
#if SWING_ARM_MACHINE == 1
	addMCommadRegulation(87, IO_PRE_ACTION, cMk, 1, NULL, NULL, NULL, NULL, AssignM87);
#endif
}

//?M87
int8 AssignM87(SAssignConfiguration* pSAssignConfig, struct SGCodeParse* pGCodeBufUnit)
{
	if (pSAssignConfig == NULL || pGCodeBufUnit == NULL)
	{
		return -1;
	}

	setIO(M87, pGCodeBufUnit->ucIOCommandArray, IO_MAXGROUP_IN_SINGLE_LINE);

}
