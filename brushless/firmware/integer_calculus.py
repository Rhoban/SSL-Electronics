#!/usr/bin/python3

import numpy as np

def unsigned_integer_digits(value):
    """
    Compute the minimal number of bits needed to encode a positive integer.

    Input :
      value : the integer

    Tests:
    >>> [ unsigned_integer_digits(i) for i in range(18) ]
    [0, 1, 2, 2, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4, 5, 5]
    >>> unsigned_integer_digits(-1)
    Traceback (most recent call last):
    ...
    AssertionError
    """
    assert( value >= 0 )
    value = int( value )
    res = 0
    while( value != 0 ):
        res += 1
        value = value//2
    return res

def signed_integer_digits(value):
    """
    Compute the minimal number of bits needed to encode a signed integer.

    Input :
      value : the integer

    Tests:
    >>> [ signed_integer_digits(i) for i in range(18) ]
    [1, 2, 3, 3, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6]
    >>> [ signed_integer_digits(-i) for i in range(18) ]
    [1, 2, 3, 3, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6]
    """
    return unsigned_integer_digits(abs(value))+1

def error_digits(error):
    """
    Compute the minimal number of bits iswitch to encode a integer with 
    respect to a given error.

    Input :
      value : the integer

    Tests:
    >>> [ error_digits(1.0/i) for i in range(1,18) ]
    [0, 1, 2, 2, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4, 5]
    >>> [ error_digits(1.0/(2**i)) for i in range(0,18) ]
    [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17]
    >>> error_digits(-1)
    Traceback (most recent call last):
    ...
    AssertionError
    >>> error_digits(0)
    Traceback (most recent call last):
    ...
    AssertionError
    """
    assert( error > 0 )
    res = 0
    while( error < 1.0 ):
        res += 1
        error *= 2
    return res

def number_of_bits(value, error):
    """
    Compute the minimal number of bits needed to encode a real with a
    signed integer with a given error.

    Input : 
      value : doube

    OUTPUT: [scale, digits] where
        scale : error made 
        digits : number of bits need to encode the value

    Tests:
    >>> [ number_of_bits(i, 1.0) for i in range(18) ]
    [[0, 1], [0, 2], [0, 3], [0, 3], [0, 4], [0, 4], [0, 4], [0, 4], [0, 5], [0, 5], [0, 5], [0, 5], [0, 5], [0, 5], [0, 5], [0, 5], [0, 6], [0, 6]]
    >>> [ number_of_bits(-i, 1.0) for i in range(18) ]
    [[0, 1], [0, 2], [0, 3], [0, 3], [0, 4], [0, 4], [0, 4], [0, 4], [0, 5], [0, 5], [0, 5], [0, 5], [0, 5], [0, 5], [0, 5], [0, 5], [0, 6], [0, 6]]
    >>> [ number_of_bits(i, 0.1) for i in range(18) ]
    [[4, 5], [4, 6], [4, 7], [4, 7], [4, 8], [4, 8], [4, 8], [4, 8], [4, 9], [4, 9], [4, 9], [4, 9], [4, 9], [4, 9], [4, 9], [4, 9], [4, 10], [4, 10]]
    >>> [ number_of_bits(-i, 0.1) for i in range(18) ]
    [[4, 5], [4, 6], [4, 7], [4, 7], [4, 8], [4, 8], [4, 8], [4, 8], [4, 9], [4, 9], [4, 9], [4, 9], [4, 9], [4, 9], [4, 9], [4, 9], [4, 10], [4, 10]]
    """
    scale = error_digits(error)
    return [ scale, signed_integer_digits(abs(value))+scale ]

class Variable:
    def __init__( self, name, minimal, maximal, error, digits ):
        """
        >>> err = 0.001
        >>> mini = -2.1
        >>> maxi = 1.2
        >>> var = Variable(
        ...    name='k', minimal=-2.1, maximal= 1.2, error=err, digits=32
        ... )
        >>> var.to_c('a')
        'a * 1024'
        >>> var.to_py('a')
        'a * 1024'
        >>> k = -2.01
        >>> int(eval(var.to_py(k)))/var.scale - k <= err
        True
        >>> var.minimal == mini
        True
        >>> var.maximal == maxi
        True
        """
        self.name = name
        self.minimal = minimal
        self.maximal = maximal
        maxi = max( abs(minimal), abs(maximal) )
        if error is None:
            nb_bits_for_constant = signed_integer_digits(maxi)
            assert( nb_bits_for_constant <= digits )
            error = 2**(nb_bits_for_constant-digits)
        [scale, minimal_nb_bits] = number_of_bits(maxi, error)
        assert( minimal_nb_bits <= digits )
        self.scale = scale
        self.value = name
        self.error = 1.0
        self.digits = digits
        if( not( self.error <= error*(2**self.scale) ) ):
            raise ValueError('Impossible to make the constant with the expected error')
    def to_c(self, v1):
        return "%s * %s"%(v1, 2**self.scale)
    def to_py(self, v1):
        return self.to_c(v1).replace('/', '//')
    def final_error(self):
        return self.error / (2**self.scale)



class Constant:
    def __init__( self, constant, error, digits ):
        """
        >>> cst = Constant( constant=3.14, error=0.1, digits=32 )
        >>> abs( cst.value/(2**cst.scale) - 3.14 ) < 0.1
        True
        >>> 1.0/(2**(cst.scale-1)) > 0.1 
        True
        >>> cst.value < 2**(cst.digits-1)
        True
        >>> cst.constant * 2**(cst.scale) < 2**(cst.digits-1)
        True
        >>> int(2**cst.scale*cst.maximal - 2**cst.scale*cst.minimal) == int(cst.error)
        True
        >>> cst.maximal >= cst.constant
        True
        >>> cst.minimal <= cst.constant
        True
        >>> cst.scale <= cst.digits
        True
        >>> cst.to_c()
        '50'
        >>> cst.to_py()
        '50'

        >>> cst = Constant( constant=3.14, error=0.01, digits=32 )
        >>> abs( cst.value/(2**cst.scale) - 3.14 ) < 0.01
        True
        >>> 1.0/(2**(cst.scale-1)) > 0.01 
        True
        >>> cst.value < 2**(cst.digits-1)
        True
        >>> cst.constant * 2**(cst.scale) < 2**(cst.digits-1)
        True
        >>> int(2**cst.scale*cst.maximal - 2**cst.scale*cst.minimal) == int(cst.error)
        True
        >>> cst.maximal >= cst.constant
        True
        >>> cst.minimal <= cst.constant
        True
        >>> cst.scale <= cst.digits
        True
        >>> cst.to_c()
        '401'
        >>> cst.to_py()
        '401'

        >>> cst = Constant( constant=3.14, error=None, digits=32 )
        >>> cst.constant * 2**(cst.scale+1) >= 2**(cst.digits-1)
        True
        >>> cst.constant * 2**(cst.scale) < 2**(cst.digits-1)
        True
        >>> cst.maximal - cst.minimal == cst.error/2**cst.scale
        True
        >>> cst.maximal >= cst.constant
        True
        >>> cst.minimal <= cst.constant
        True
        >>> cst.scale <= cst.digits
        True
        >>> cst.to_c()
        '1685774663'
        >>> cst.to_py()
        '1685774663'
        """
        self.constant = constant
        if error is None:
            nb_bits_for_constant = signed_integer_digits(int(constant))
            assert( nb_bits_for_constant <= digits )
            error = 2**(nb_bits_for_constant-digits)
        [scale, minimal_nb_bits] = number_of_bits(constant, error)
        assert( minimal_nb_bits <= digits )
        self.scale = scale
        self.value = int( constant*(2**self.scale) )
        self.minimal = (1.0*self.value)/(2**self.scale)
        self.maximal = (self.value+1.0)/(2**self.scale)
        self.error = 1.0
        self.digits = digits
        if( not( self.error <= error*(2**self.scale) ) ):
            raise ValueError('Impossible to make the constant with the expected error')
    def to_c(self):
        return "%s"%(self.value)
    def to_py(self):
        return self.to_c().replace('/', '//')
    def final_error(self):
        return self.error / (2**self.scale)

class Add:
    def __init__( self, term_1, term_2, error, digits ):
        """
        >>> c1 = 3.14
        >>> c2 = 6.0
        >>> err = 0.01
        >>> cst_1 = Constant( constant=c1, error=None, digits=32 )
        >>> cst_2 = Constant( constant=c2, error=None, digits=32 )
        >>> add = Add( term_1=cst_1, term_2=cst_2, error=err, digits=32)
        >>> v_1 = cst_1.value
        >>> v_2 = cst_2.value
        >>> add.to_c('a', 'b')
        'b/2 + a/4'
        >>> add.to_py('a', 'b')
        'b//2 + a//4'
        >>> abs( eval(add.to_py(v_1, v_2))/(2**add.scale) - (c1+c2) ) <= err
        True

        >>> cst_1 = Constant( constant=c1, error=None, digits=32 )
        >>> cst_2 = Constant( constant=c2, error=None, digits=11 )
        >>> add = Add( term_1=cst_1, term_2=cst_2, error=err, digits=32)
        >>> v_1 = cst_1.value
        >>> v_2 = cst_2.value
        >>> add.to_c('a', 'b')
        'b*1048576 + a/4'
        >>> add.to_py('a', 'b')
        'b*1048576 + a//4'
        >>> abs( eval(add.to_py(v_1, v_2))/(2**add.scale) - (c1+c2) ) <= err
        True

        >>> cst_1 = Constant( constant=c1, error=None, digits=32 )
        >>> cst_2 = Constant( constant=c2, error=None, digits=10 )
        >>> add = Add( term_1=cst_1, term_2=cst_2, error=err, digits=32)
        Traceback (most recent call last):
        ...
        ValueError: Impossible to make the addition with the expected error

        >>> cst_1 = Constant( constant=c1, error=None, digits=11 )
        >>> cst_2 = Constant( constant=c2, error=None, digits=32 )
        >>> add = Add( term_1=cst_1, term_2=cst_2, error=err, digits=32)
        >>> v_1 = cst_1.value
        >>> v_2 = cst_2.value
        >>> add.to_c('a', 'b')
        'a*524288 + b/2'
        >>> add.to_py('a', 'b')
        'a*524288 + b//2'
        >>> abs( eval(add.to_py(v_1, v_2))/(2**add.scale) - (c1+c2) ) <= err
        True

        >>> cst_1 = Constant( constant=c1, error=None, digits=11 )
        >>> cst_2 = Constant( constant=c2, error=None, digits=32 )
        >>> add = Add( term_1=cst_1, term_2=cst_2, error=err, digits=30)
        >>> v_1 = cst_1.value
        >>> v_2 = cst_2.value
        >>> add.to_c('a', 'b')
        'a*131072 + b/8'
        >>> add.to_py('a', 'b')
        'a*131072 + b//8'
        >>> abs( eval(add.to_py(v_1, v_2))/(2**add.scale) - (c1+c2) ) <= err
        True
        """
        self.digits = digits
        self.swap_terms = False
        if term_1.scale > term_2.scale :
            term_1, term_2 = term_2, term_1
            self.swap_terms = True
        self.maximal = term_1.maximal + term_2.maximal
        self.minimal = term_1.minimal + term_2.minimal
        self.scale_diff = term_2.scale - term_1.scale
        max_value = max(
            abs(
                (2**self.scale_diff) * term_1.maximal * 2**term_1.scale +
                term_2.maximal * 2**term_2.scale
            ),
            abs(
                (2**self.scale_diff) * term_1.minimal * 2**term_1.scale +
                term_2.minimal * 2**term_2.scale
            ),
        )
        self.k = unsigned_integer_digits( max_value/2**(digits-1) )
        self.error = (
            (2**self.scale_diff) * term_1.error + term_2.error -1 
        )/(2**self.k) + 1
        self.scale = term_2.scale - self.k
        if( not error is None and not( self.error <= error* (2**self.scale) ) ):
            raise ValueError('Impossible to make the addition with the expected error')
    def to_c(self, variable_1, variable_2):
        if self.swap_terms:
            variable_1, variable_2 = variable_2, variable_1
        den_b = 2**self.k
        min_scale = min(self.k, self.scale_diff)
        num_a = 2**(self.scale_diff - min_scale)
        den_a = 2**(self.k - min_scale)
        if( num_a == 1 ):
            return "%s/%s + %s/%s"%(variable_1, den_a, variable_2, den_b)
        else:
            return "%s*%s + %s/%s"%(variable_1, num_a, variable_2, den_b)
    def to_py(self, variable_1, variable_2):
        return self.to_c(variable_1, variable_2).replace('/', '//')
    def final_error(self):
        return self.error / (2**self.scale)

class Mult:
    def __init__(self, term_1, term_2, error, digits):
        """
        >>> c1 = 3.14
        >>> c2 = 6.0
        >>> err = 0.01
        >>> cst_1 = Constant( constant=c1, error=None, digits=32 )
        >>> cst_2 = Constant( constant=c2, error=None, digits=32 )
        >>> mul = Mult( term_1=cst_1, term_2=cst_2, error=err, digits=32)
        >>> v_1 = cst_1.value
        >>> v_2 = cst_2.value
        >>> mul.to_c('a', 'b')
        '(a/65536)*(b/32768)'
        >>> mul.to_py('a', 'b')
        '(a//65536)*(b//32768)'
        >>> abs( eval(mul.to_py(v_1, v_2))/(2**mul.scale) - (c1*c2) ) <= err
        True
        """
        self.digits = digits
        extremum_1 = [
            term_1.maximal * 2**term_1.scale, term_1.minimal * 2**term_1.scale
        ]
        extremum_2 = [
            term_2.maximal * 2**term_2.scale, term_2.minimal * 2**term_2.scale
        ]
        support = [ (u1,u2) for u1 in extremum_1 for u2 in extremum_2 ]
        self.minimal = min( [ u1*u2 for (u1,u2) in support ] )
        self.maximal = max( [ u1*u2 for (u1,u2) in support ] )
        max_value = max( [ abs(u1)*abs(u2) for (u1,u2) in support ] )

        self.k = unsigned_integer_digits( max_value/2**(digits-1) )
        t_min = None
        for i in range(self.k):
            j = self.k - i
            t_max = max( [
                abs(int(u1))//2**i + abs(int(u2))//2**j 
                for (u1, u2) in support
            ] )
            if t_min is None or t_max < t_min:
                t_min = t_max
                self.i = i
                self.j = j
        self.scale = term_1.scale + term_2.scale - self.k
        max_error = max( [
                term_1.error * int(u2) + term_2.error * int(u1) +
                term_1.error * term_2.error
                for (u1, u2) in support
        ] )
        self.error = t_min + max_error/2**self.k
        if( not error is None and not( self.error <= error*(2**self.scale) ) ):
            raise ValueError('Impossible to make the constant with the expected error')
    def to_c(self, variable_1, variable_2):
        return "(%s/%s)*(%s/%s)"%(
            variable_1, 2**self.i, variable_2, 2**self.j
        )
    def to_py(self, variable_1, variable_2):
        return self.to_c(variable_1, variable_2).replace('/', '//')
if __name__ == "__main__":
    import doctest
    doctest.testmod()
