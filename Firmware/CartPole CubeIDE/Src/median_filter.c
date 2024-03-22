/**********************************************************************/
/* Freeware Demo SW from the www.dsp-weimich.com                      */
/* Version 1.0                                                        */
/* median_filter.c                                                    */
/* Support the article Median Filter                                  */
/**********************************************************************/

#include "median_filter.h"
#include <stdbool.h>

/*
 Insertion Sort Method
 Input
 int* ptrArray  - pointer on the working array
 unsigned int  arraySize - array length
 Return
 void - nothing
*/
void InsertionSortMethod(int* ptrArray, unsigned int arraySize)
{
	unsigned char exit_flag;
	int help_value;
	unsigned int i_index;
	unsigned int j_index = 1u;
	do
	{/* Main Loop: j = 1,...N-1 */
		i_index = j_index - 1;
		help_value = ptrArray[j_index];
		exit_flag = false;
		do
		{/* Loop: i= j-1, j-2,..., 0 */
			if(help_value >= ptrArray[i_index])
			{/* Insert the A[j] in the suitable position */
				ptrArray[i_index+1] = help_value;
				exit_flag = true;
			}
			else
			{/* Shift the A[i] to A[i+1] */
				ptrArray[i_index+1] = ptrArray[i_index];
				if(0 == i_index)
				{/* First array element */
					ptrArray[i_index] = help_value;
					exit_flag = true;
				}
				else
				{/* i = j-1,..., 0 */
					i_index--;
				}
			}
		} while(false == exit_flag);
		j_index++;
	} while(j_index < arraySize);
}

/*
 Classic Median Filter
 Input
 int* ptrArray  - pointer on the input array
 unsigned int  arraySize - array length
 Return
 int - filter output
*/
int ClassicMedianFilter(int* ptrArray, unsigned int arraySize)
{
	int returnValue;
	
	int ArrayCopy[arraySize];

	for (int i = 0; i < arraySize; i++) {
		ArrayCopy[i] = ptrArray[i];
	    }

	if(arraySize > 1u)
	{
		InsertionSortMethod(ArrayCopy,arraySize);
		if(0 == (arraySize&1u))
		{/* Even number */
			returnValue = (ArrayCopy[(arraySize/2)-1]+ArrayCopy[arraySize/2])/2;
		}
		else
		{/* Odd number */
			returnValue = ArrayCopy[(arraySize-1)/2];
		}
	}
	else
	{/* Array has only one element */
		returnValue = ArrayCopy[0];
	}
	return returnValue;
}






void InsertionSortMethod_float(float* ptrArray, unsigned int arraySize)
{
	unsigned char exit_flag;
	float help_value;
	unsigned int i_index;
	unsigned int j_index = 1u;
	do
	{/* Main Loop: j = 1,...N-1 */
		i_index = j_index - 1;
		help_value = ptrArray[j_index];
		exit_flag = false;
		do
		{/* Loop: i= j-1, j-2,..., 0 */
			if(help_value >= ptrArray[i_index])
			{/* Insert the A[j] in the suitable position */
				ptrArray[i_index+1] = help_value;
				exit_flag = true;
			}
			else
			{/* Shift the A[i] to A[i+1] */
				ptrArray[i_index+1] = ptrArray[i_index];
				if(0 == i_index)
				{/* First array element */
					ptrArray[i_index] = help_value;
					exit_flag = true;
				}
				else
				{/* i = j-1,..., 0 */
					i_index--;
				}
			}
		} while(false == exit_flag);
		j_index++;
	} while(j_index < arraySize);
}



float ClassicMedianFilter_float(float* ptrArray, unsigned int arraySize)
{
	float returnValue;

	float ArrayCopy[arraySize];

	for (int i = 0; i < arraySize; i++) {
		ArrayCopy[i] = ptrArray[i];
	    }

	if(arraySize > 1u)
	{
		InsertionSortMethod_float(ArrayCopy,arraySize);
		if(0 == (arraySize&1u))
		{/* Even number */
			returnValue = (ArrayCopy[(arraySize/2)-1]+ArrayCopy[arraySize/2])/2;
		}
		else
		{/* Odd number */
			returnValue = ArrayCopy[(arraySize-1)/2];
		}
	}
	else
	{/* Array has only one element */
		returnValue = ArrayCopy[0];
	}
	return returnValue;
}
