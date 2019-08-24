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

  return 0;
}
