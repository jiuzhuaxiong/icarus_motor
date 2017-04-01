#include <stdio.h>
#include <cstring>
#include <stdlib.h>

#define ROTATION = 117;
#define FACTOR = 1000;

bool parseCmd(char* in, float& r, float& v);
bool isNaN(const float& val);
char* subString (const char* input, int offset, int len, char* dest);

