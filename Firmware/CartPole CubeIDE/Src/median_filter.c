/**********************************************************************/
/* Freeware Demo SW from the www.dsp-weimich.com                      */
/* Version 1.0                                                        */
/* median_filter.c                                                    */
/* Support the article Median Filter                                  */
/**********************************************************************/

#include "median_filter.h"

/*
 Insertion Sort Method
 Input
 sint16* ptrArray  - pointer on the working array
 uint16  arraySize - array length
 Return
 void - nothing
*/
void InsertionSortMethod(sint16* ptrArray, uint16 arraySize)
{
	uint8 exit_flag;
	sint16 help_value;
	uint16 i_index;
	uint16 j_index = 1u;
	do
	{/* Main Loop: j = 1,...N-1 */
		i_index = j_index - 1;
		help_value = ptrArray[j_index];
		exit_flag = FALSE;
		do
		{/* Loop: i= j-1, j-2,..., 0 */
			if(help_value >= ptrArray[i_index])
			{/* Insert the A[j] in the suitable position */
				ptrArray[i_index+1] = help_value;
				exit_flag = TRUE;
			}
			else
			{/* Shift the A[i] to A[i+1] */
				ptrArray[i_index+1] = ptrArray[i_index];
				if(0 == i_index)
				{/* First array element */
					ptrArray[i_index] = help_value;
					exit_flag = TRUE;
				}
				else
				{/* i = j-1,..., 0 */
					i_index--;
				}
			}
		} while(FALSE == exit_flag);
		j_index++;
	} while(j_index < arraySize);
}

/*
 Classic Median Filter
 Input
 sint16* ptrArray  - pointer on the input array
 uint16  arraySize - array length
 Return
 sint16 - filter output
*/
sint16 ClassicMedianFilter(sint16* ptrArray, uint16 arraySize)
{
	sint16 returnValue;
	
	if(arraySize > 1u)
	{
		InsertionSortMethod(ptrArray,arraySize);
		if(0 == (arraySize&1u))
		{/* Even number */
			returnValue = (ptrArray[(arraySize/2)-1]+ptrArray[arraySize/2])/2;
		}
		else
		{/* Odd number */
			returnValue = ptrArray[(arraySize-1)/2];
		}
	}
	else
	{/* Array has only one element */
		returnValue = ptrArray[0];
	}
	return returnValue;
}

/*
 Advance Median Filter
 Input
 sint16* ptrArray  - pointer on the input array
 uint16  arraySize - array length
 Return
 sint16 - filter output
*/
#define ADVANCE_MEDIAN_DELTA  36 /* Only example value */
sint16 AdvanceMedianFilter(sint16* ptrArray, uint16 arraySize)
{
	sint16 returnValue;
	sint16 helpValue;
	sint32 sumValue;
	uint32 numberValue;
	sint32 deltaValue;
	uint16 indexLoop;
	
	helpValue = ClassicMedianFilter(ptrArray,arraySize);
	
	for(indexLoop=0, sumValue=0, numberValue=0; indexLoop < arraySize; indexLoop++)
	{
		if(helpValue >= ptrArray[indexLoop])
		{
			deltaValue = helpValue - ptrArray[indexLoop];
		}
		else
		{
			deltaValue = ptrArray[indexLoop] - helpValue;
		}
		if(deltaValue <= ADVANCE_MEDIAN_DELTA)
		{
			sumValue += ptrArray[indexLoop];
			numberValue++;
		}
	}
	if(0 == numberValue)
	{
		returnValue = helpValue;
	}
	else
	{
		returnValue = sumValue / numberValue;
	}
	return returnValue;
}
