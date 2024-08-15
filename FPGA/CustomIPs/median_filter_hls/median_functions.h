#ifndef MEDIAN_FUNCTIONS_H
#define MEDIAN_FUNCTIONS_H

#define MAX_WINDOW_SIZE 64

unsigned short get_median(unsigned short newValue, short window_size);

int remove_oldest_value(short window_size);
int insert_new_value(unsigned short newValue, short window_size);
int calculate_median(unsigned short &median);

#endif // MEDIAN_FUNCTIONS_H
