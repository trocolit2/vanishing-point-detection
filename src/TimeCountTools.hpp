#include <ctime>
#include <iostream>

#define CAT(prefix, suffix)            prefix ##suffix
#define _UNIQUE_LABEL(prefix, suffix)  CAT(prefix, suffix)
#define UNIQUE_LABEL(prefix)           _UNIQUE_LABEL(prefix, __LINE__)

#define TIME_COUNT(var_time) \
  clock_t UNIQUE_LABEL(begin_time) = std::clock(); \
  goto UNIQUE_LABEL(code_block); \
  while(1) \
    if (1) { \
      var_time = double(std::clock() - UNIQUE_LABEL(begin_time))/CLOCKS_PER_SEC; \
      break;} \
    else UNIQUE_LABEL(code_block):
