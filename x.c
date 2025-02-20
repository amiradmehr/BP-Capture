#include <stdio.h>

// Define an enumeration for days of the week
typedef enum
{
  MONDAY,
  TUESDAY,
  WEDNESDAY,
  THURSDAY,
  FRIDAY,
  SATURDAY,
  SUNDAY
} Day;

int main()
{
  // Declare a variable of type Day
  Day today = SUNDAY;

  // Print the value of today
  printf("Today is day number: %d\n", today);

  return 0;
}