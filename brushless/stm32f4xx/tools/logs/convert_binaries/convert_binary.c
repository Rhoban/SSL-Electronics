/*
 * Copyright (C) 2019  Adrien Boussicault <adrien.boussicault@labri.fr>
 *
 * This program is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation, either
 * version 3 of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this program.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <stdbool.h>
#include <float.h>
#include <string.h>
#include <math.h>

uint32_t convert_to_uint32(const char* s){
  uint32_t res = 0;
  for( uint32_t i=0; i<8; i++ ){
    res |= ( ( ( s[i] - '0' ) & 0x0000000F ) << 4*(7-i) );
  }
  return res;
}

typedef union {
  uint32_t from;
  int32_t to;
} uint32_to_int32_t;

int32_t convert_to_int32(const char* s){
  uint32_to_int32_t val;
  val.from = convert_to_uint32(s);
  return val.to;
}

typedef union {
  uint32_t from;
  float to;
} uint32_to_float_t;

float convert_to_float(const char* s){
  uint32_to_float_t val;
  val.from = convert_to_uint32(s);
  return val.to;
}

#define NB_UINT32 8
uint32_t test_uint32_t[NB_UINT32] = {
0, 1, 2, 16384, 16385, 10387, 4294967294, 4294967295
};
const char * test_uint32_string[NB_UINT32] = {
  "00000000",
  "00000001",
  "00000002",
  "00004000",
  "00004001",
  "00002893",
  "\?\?\?\?\?\?\?>",
  "\?\?\?\?\?\?\?\?"
};

#define NB_INT32 9
int32_t test_int32_t[NB_INT32] = {
  0, 1, -1, 16384, -16384, 16385, 10387, -2147483648, 2147483647
};

const char * test_int32_string[NB_INT32] = {
  "00000000",
  "00000001",
  "\?\?\?\?\?\?\?\?",
  "00004000",
  "\?\?\?\?<000",
  "00004001",
  "00002893",
  "80000000",
  "7\?\?\?\?\?\?\?"
};

#define NB_FLOAT 26
float test_float[NB_FLOAT] = {
  0, 1, 2, 4, 0.1, 0.2, 0.4, 0.125, 0.5, 1234.567, -0, -1, -2, -4, -0.1, -0.2, -0.4, -0.125, -0.5, -1234.567, 1.17549435e-38, 5.9e-39, 1.4013e-45, 1.17549421e-38, 1.23456e-12, 3.40282326e+38
};

const char * test_float_string[NB_FLOAT] = {
  "00000000",
  "3\?800000",
  "40000000",
  "40800000",
  "3=<<<<<=",
  "3>4<<<<=",
  "3><<<<<=",
  "3>000000",
  "3\?000000",
  "449:5225",
  "80000000",
  ";\?800000",
  "<0000000",
  "<0800000",
  ";=<<<<<=",
  ";>4<<<<=",
  ";><<<<<=",
  ";>000000",
  ";\?000000",
  "<49:5225",
  "00800000",
  "00403><=",
  "00000001",
  "007\?\?\?\?\?",
  "2;:=;\?;6",
  "7\?7\?\?\?\?>"
};


bool test(){
  for( uint32_t i=0; i<NB_UINT32; i++){
    assert( test_uint32_t[i] == convert_to_uint32(test_uint32_string[i]) );
  }
  for( uint32_t i=0; i<NB_INT32; i++){
    assert( test_int32_t[i] == convert_to_int32(test_int32_string[i]) );
  }
  for( uint32_t i=0; i<NB_FLOAT; i++){
    assert( test_float[i] == convert_to_float(test_float_string[i]) );
  }
  return true;
}

int main( int argc, char* argv[]){
  if( argc != 2 ){
    fprintf(stderr, "Invalid number of parameter.\n");
    fprintf(stderr, "Usage: %s <log_path>\n", argv[0]);
    fprintf(stderr, "       %s -test\n", argv[0]);
    return 1;
  }

  if( !strcmp( argv[1], "-test") ){
    if( test() ){
      return 0;
    }else{
      return 1;
    }
  }

  const char* file_path = argv[1];

  #define SIZE_LINE 4096
  FILE* file = fopen(file_path, "r"); /* should check the result */
  if( !file ){
    fprintf(stderr, "Impossible to open the file : %s.\n", file_path);
    return 1;
  }

  char names[SIZE_LINE];
  char types[SIZE_LINE];
  char line[SIZE_LINE];

  if( !fgets(names, SIZE_LINE, file) ){
    fprintf(stderr, "An error occurs when reading the first line.\n");
    return 1;
  };
  printf("%s", names);

  if( !fgets(types, SIZE_LINE, file) ){
    fprintf(stderr, "An error occurs when reading the second line.\n");
    return 1;
  }
  int N=0;
  char* token;
  char* saveptr;
  for (char* str=types; ; str = NULL) {
    token = strtok_r(str, "' ", &saveptr);
    if (token == NULL) break;
    types[N] = *token;
    N++;
  } 

  while (fgets(line, sizeof(line), file)) {
    for( int i=0; i<N; i++ ){
      switch( types[i] ){
        case 'u':
          fprintf( stdout, "%u", convert_to_uint32(line+i*8));
          break;  
        case 'f':
          fprintf(stdout, "%.7g", convert_to_float(line+i*8));
          break;  
        case 'i':
          fprintf(stdout, "%d", convert_to_int32(line+i*8));
          break;  
        default:
          assert(false);
          break;
      }
      if( i<N-1 ){
        fprintf(stdout, ", ");
      }else{
        fprintf(stdout, "\n");
      }
    }
  }
  fclose(file);

  return 0;
}
