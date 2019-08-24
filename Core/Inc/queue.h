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

#pragma once

#include "assertion.h" 
#include <tools.h>
#include <stdint.h>

#define define_queue_headers(type, name, size) \
  typedef struct { \
    uint32_t head; \
    uint32_t tail; \
    \
    _Static_assert(IS_POW_2(size), "queue size have to be a 2^N.") \
    type queue[size]; \
  } name ## _queue_t; \
  \
  inline _Bool name ## _is_empty(name ## _queue_t * name){ \
    return name->head == name->tail; \
  } \
  inline _Bool name ## _is_full(name ## _queue_t * name){ \
    return NEXT(name->head, size) == name->tail; \
  } \
  inline void name ## _append(name ## _queue_t * name, type e){ \
    if( name ## _is_full(name) ) return; \
    name->queue[name->head] = e; \
    name->head = NEXT(name->head, size); \
  } \
  inline void name ## _fill(name ## _queue_t * name, type* buf, uint32_t len){ \
    for( uint32_t i=0; i<len; i++ ){ \
      name ## _append(name, buf[i]); \
    } \
  } \
  inline type name ## _pop(name ## _queue_t * name){ \
    type tail = name->queue[name->tail]; \
    name->tail = NEXT(name->tail, size); \
    return tail; \
  } \
  inline void name ## _collect(name ## _queue_t * name, type* buffer, uint32_t* len){ \
    *len = 0; \
    while(!name ## _is_empty(name)){ \
      buffer[*len] = name ## _pop(name); \
      (*len)++; \
    } \
  } \
  inline void name ## _process(name ## _queue_t * name, void (*fct)(type e, void* data), void* data){ \
    while( ! name ## _is_empty(name) ){ \
      fct( name ## _pop(name), data ); \
    } \
  } \
  inline void name ## _clear(name ## _queue_t * name){ \
    name->tail = name->head; \
  } \
  inline uint32_t name ## _size(name ## _queue_t * name){ \
    return (name->head - name->tail) & (size-1); \
  }

#define declare_static_queue(name) static name ## _queue_t name = {0, 0};

#define define_and_declare_static_queue(type, name, size) \
  define_queue_headers(type, name, size) \
  declare_static_queue(name);
  
