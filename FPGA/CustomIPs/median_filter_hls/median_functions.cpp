#include "median_functions.h"
#include <stdlib.h>

static unsigned short window[MAX_WINDOW_SIZE];
static unsigned short buffer[MAX_WINDOW_SIZE];
static short currentWindowSize = 0;
static short bufferIndex = 0;

static void insertIntoSortedWindow(unsigned short newvalue) {
  short i;
  short i_insert = 0;
  short inserted = 0;
  for (i = MAX_WINDOW_SIZE - 2; i >= 0; i--) {

	  if(i < currentWindowSize ){
	      if (window[i] > newvalue) {
	          window[i + 1] = window[i];
	      } else if (inserted == 0) {
	          inserted =  1;
	          i_insert = i+1;
	      }
	  }


  }
  window[i_insert] = newvalue;
}



static void removeFromSortedWindow(unsigned short oldvalue) {
    short i;
    short i_remove = currentWindowSize;
    short i_remove_found = 0;
    for (i = 0; i < MAX_WINDOW_SIZE; i++) {


        if(i < currentWindowSize - 1)
        {
          if (window[i] == oldvalue) {
              i_remove_found = 1;
          }
          if (i_remove_found == 1){
            window[i] = window[i + 1];
          }
        }

    }
}

// New function to remove the oldest value
int remove_oldest_value(short window_size) {
    if (currentWindowSize == window_size) {
        removeFromSortedWindow(buffer[bufferIndex]);
        currentWindowSize--;
    } else if (currentWindowSize > window_size) { // To make the window shorter, you need to reset the median. Gradual decrease would introduce too much latency
    	currentWindowSize = 0;
    	bufferIndex = 0;
    }
    return 1;
}

// New function to insert a new value
int insert_new_value(unsigned short newValue, short window_size) {
    insertIntoSortedWindow(newValue);
    buffer[bufferIndex] = newValue;
    bufferIndex = (bufferIndex + 1) % window_size;
    currentWindowSize++;
    return 1;
}

int calculate_median(unsigned short &median) {
    if (currentWindowSize % 2 != 0) {
        median = window[currentWindowSize / 2];
    } else {
        median = (window[(currentWindowSize - 1) / 2] + window[currentWindowSize / 2]) / 2;
    }
    return 1;
}


unsigned short get_median(unsigned short newValue, short window_size) {

	unsigned short median;
    remove_oldest_value(window_size);
    insert_new_value(newValue, window_size);
    calculate_median(median);
    return median;
}
