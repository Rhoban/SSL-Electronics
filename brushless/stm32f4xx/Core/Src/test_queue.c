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

#include <assert.h>
#include <assertion.h>
#include <queue.h>
#include <stdint.h>
#include <stdbool.h>

#define SIZE 32
define_and_declare_static_queue(uint32_t, test, SIZE)
define_and_declare_static_queue(uint32_t, test_2, SIZE)

static int test_int = 42;
static int cpt = 0;
                         
void test_fct(uint32_t e, void *data){
  assert( data == &test_int);
  assert( e==cpt );
  cpt++;
}

int main(){
  {
    assert( test_is_empty(&test) );
    assert( ! test_is_full(&test) );
    test_append(&test, 0);
    assert( !test_is_empty(&test) );
    test_append(&test, 1);
    assert( !test_is_empty(&test) );
    test_append(&test, 2);
    assert( !test_is_empty(&test) );
    test_append(&test, 3);
    assert( !test_is_empty(&test) );
    test_append(&test, 4);
    assert( !test_is_empty(&test) );
    assert( test_pop(&test) == 0 );
    assert( !test_is_empty(&test) );
    assert( test_pop(&test) == 1 );
    assert( !test_is_empty(&test) );
    assert( test_pop(&test) == 2 );
    assert( !test_is_empty(&test) );
    assert( test_pop(&test) == 3 );
    assert( !test_is_empty(&test) );
    assert( test_pop(&test) == 4 );
    assert( test_is_empty(&test) );
    assert( ! test_is_full(&test) );
  }
  {
    #define OFFSET 42
    assert( test_is_empty(&test) );
    for( int i = 0; i<SIZE-1; i++ ){
      test_append(&test, OFFSET+i);
    }
    assert( test_is_full(&test) );
    test_clear(&test);
    assert( test_is_empty(&test) );
    for( int i = 0; i<2*SIZE; i++ ){
      test_append(&test, OFFSET+i);
    }
    for( int i = 0; i<SIZE-1; i++ ){
      assert( test_pop(&test) == OFFSET+i );
    }
    assert( test_is_empty(&test) );
  }
  {
    test_2_append(&test_2, 101);
    test_2_append(&test_2, 102);
    test_2_append(&test_2, 103);
    test_2_append(&test_2, 104);
    test_2_pop(&test_2);
    test_2_pop(&test_2);
    test_2_pop(&test_2);
    test_2_pop(&test_2);
    
    assert( test_2_is_empty(&test_2) );
    for( int i = 0; i<SIZE-1; i++ ){
      test_2_append(&test_2, i);
    }
    test_2_process(&test_2, test_fct, &test_int);
    assert(cpt == SIZE-1);
  }
  {
    test_2_clear(&test_2);
    test_2_append(&test_2, 201);
    test_2_append(&test_2, 202);
    test_2_append(&test_2, 203);
    test_2_append(&test_2, 204);
    test_2_append(&test_2, 205);
    test_2_append(&test_2, 206);
    test_2_append(&test_2, 207);
    test_2_drop(&test_2, 0); 
    assert( test_2_size(&test_2) == 7 );
    test_2_drop(&test_2, 3); 
    assert( test_2_size(&test_2) == 4 );
    assert( test_2_pop(&test_2) == 204 );
    assert( test_2_pop(&test_2) == 205 );
    assert( test_2_pop(&test_2) == 206 );
    assert( test_2_pop(&test_2) == 207 );
    assert( test_2_size(&test_2) == 0 );
    assert( test_is_empty(&test) );
  }

  return 0;
}
