#!/usr/bin/python3

from integer_calculus import *

digits = 32

print("--------------")
print("Evaluation of calculus errors")
print("--------------")

k_pos_p = Variable( minimal=0.1, maximal=10.0, error=None, digits=digits )
print( "k_pos_p : " + str( k_pos_p.final_error() ) )
k_pos_i = Variable( minimal=0.001, maximal=0.1, error=None, digits=digits )
print( "k_pos_i : " + str( k_pos_i.final_error() ) )
#k_pos_d = Variable( minimal=0.1, maximal=10.0, error=None, digits=digits )
#print( "k_pos_d : " + str( k_pos_d.final_error() ) )

k_speed_p = Variable( minimal=0.1, maximal=10.0, error=None, digits=digits )
print( "k_speed_p : " + str( k_speed_p.final_error() ) )
k_speed_i = Variable( minimal=0.001, maximal=0.1, error=None, digits=digits )
print( "k_speed_i : " + str( k_speed_i.final_error() ) )
#k_speed_d = Variable( minimal=0.1, maximal=10.0, error=None, digits=digits )
#print( "k_speed_d : " + str( k_speed_d.final_error() ) )

dt = Constant(constant=1.0/3000.0, error=None, digits=digits)
print( "dt : " + str( dt.final_error() ) )
inv_dt = Constant(constant=3000.0, error=None, digits=digits)
print( "inv_dt : " + str( inv_dt.final_error() ) )

theta = Input( minimal=-6000.0, maximal=6000.0, scale=14, digits=digits )
print( "theta : " + str( theta.final_error() ) )
theta_c = Variable( minimal=-6000.0, maximal=6000.0, error=None, digits=digits )
print( "theta_c : " + str( theta_c.final_error() ) )

neg_theta_c = Neg(theta_c)
print( "neg_theta_c : " + str( neg_theta_c.final_error() ) )

theta_error = Add( term_1=theta, term_2=neg_theta_c, error=None, digits=digits );
print( "theta_error : " + str( theta_error.final_error() ) )

minimal_theta_error=-10.0
maximal_theta_error=10.0

theta_error_with_histeresis = Limit( 
    term = theta_error,
    minimal = minimal_theta_error,
    maximal = maximal_theta_error
)
print( "theta_error_with_histeresis : " + str( theta_error_with_histeresis.final_error() ) )

speed_p = Mult( term_1=theta_error_with_histeresis, term_2=k_pos_p, error=None, digits=digits )
#print( "t_min : " + str( speed_p.t_min/2**speed_p.scale ) )
#print( "max_error : " + str( speed_p.max_error/2**speed_p.scale ) )
#print( "k : " + str( speed_p.k ) )
print( "speed_p : " + str( speed_p.final_error() ) )

speed_load_1 = Mult(
    term_1=theta_error_with_histeresis, 
    term_2=k_pos_i, 
    error=None, digits=digits
)
print( "speed_load_1 : " + str( speed_load_1.final_error() ) )
speed_load = Mult(
    term_1 = speed_load_1,
    term_2 = dt,
    error=None, digits=digits 
)
minimal_speed = -20.0
maximal_speed = 20.0
print( "speed_load : " + str( speed_load.final_error() ) )
speed_i = Accumulator(
    load=speed_load, minimal=minimal_speed, maximal=maximal_speed,
    digits=digits
)
print( "speed_i : " + str( speed_i.final_error() ) )

#speed_d = Mult(
#    Mult( term_1=theta_error, term_2=k_pos_d, error=None, digits=digits ),
#    inv_dt, error=None, digits=digits 
#)
#print( "speed_d : " + str( speed_d.final_error() ) )

speed_c = Add(
    term_1 = speed_p,
    term_2 = speed_i,
    error=None, digits=digits
)
print( "speed_c : " + str(speed_c.final_error()) )

speed = Input( minimal=-50.0, maximal=50.0, scale=14, digits=digits )
print( "speed : " + str( speed.final_error() ) )

neg_speed_c = Neg(speed_c)
print( "neg_speed_c : " + str( neg_speed_c.final_error() ) )

speed_error = Add(
    term_1=speed,
    term_2=neg_speed_c,
    error=None, digits=digits
)
print( "speed_error : " + str( speed_error.final_error() ) )

minimal_speed_error = -30
maximal_speed_error = 30
speed_error_with_histeresis = Limit(
    term = speed_error,
    minimal = minimal_speed_error,
    maximal = maximal_speed_error
)
print( "speed_error_with_histeresis : " + str( speed_error_with_histeresis.final_error() ) )

voltage_p = Mult(
    term_1 = speed_error_with_histeresis,
    term_2 = k_speed_p,
    error = None,
    digits = digits
)
print( "voltage_p error : " + str( voltage_p.final_error() ) )


voltage_load_1 = Mult(
    term_1=speed_error_with_histeresis, term_2=k_speed_i, 
    error=None, digits=digits
)
print( "voltage_load_1 error : " + str( voltage_load_1.final_error() ) )
voltage_load = Mult(
    term_1 = voltage_load_1,
    term_2 = dt,
    error=None, digits=digits 
)

reference_voltage = 1024
minimal_voltage = -reference_voltage
maximal_voltage = reference_voltage
print( "voltage_load error : " + str( voltage_load.final_error() ) )
voltage_i = Accumulator(
    load=voltage_load,
    minimal=minimal_voltage, maximal=maximal_voltage,
    digits=digits
)
print( "voltage_i error : " + str( voltage_i.final_error() ) )

voltage_q = Add(
    term_1 = voltage_p,
    term_2 = voltage_i,
    error = None,
    digits = digits
)
print( "voltage_q error : " + str( voltage_q.final_error() ) )

reference_voltage_q = Scale(
    term=voltage_q, minimal=-reference_voltage, maximal=reference_voltage, digits=32
)
print( "reference_voltage_q error : " + str( reference_voltage_q.final_error() ) )
assert( reference_voltage_q.final_error()< 0.5 );

print("===========================")
print("Complete calculus")
print("===========================")

print("k_pos_p")
print(k_pos_p.to_c('i_k_pos_p'))

print("k_pos_i")
print(k_pos_i.to_c('i_k_pos_i'))

print("k_speed_p")
print(k_speed_p.to_c('i_k_speed_p'))

print("k_speed_i")
print(k_speed_i.to_c('i_k_speed_i'))

print("dt")
print(dt.to_c())

print("inv_dt")
print(inv_dt.to_c())

print("theta")
print(theta.to_c('theta'))

print("theta_c")
print(theta_c.to_c('i_theta_c'))

print("neg_theta_c")
print(neg_theta_c.to_c('theta_c'))

print("theta_error")
print(theta_error.to_c('theta', 'neg_theta_c'))

print("theta_error_with_histeresis")
print(theta_error_with_histeresis.to_c('theta_error'))

print("speed_p")
print(speed_p.to_c('theta_error_with_histeresis', 'k_pos_p' ))

print("speed_load_1")
print(speed_load_1.to_c('theta_error_with_histeresis', 'k_pos_i'))

print("speed_load")
print(speed_load.to_c('speed_load_1', 'dt'))

print("speed_i")
print(speed_i.to_c('speed_i', 'speed_load'))

print("speed_c")
print(speed_c.to_c('speed_p', 'speed_i'))

print("speed")
print(speed.to_c("speed"))

print("neg_speed_c")
print(neg_speed_c.to_c("speed_c"))

print("speed_error")
print(speed_error.to_c("speed", "neg_speed_c"))

print("speed_error_with_histeresis")
print(speed_error_with_histeresis.to_c("speed_error"))

print("voltage_p")
print(voltage_p.to_c("speed_error_with_histeresis", "k_speed_p"))

print("voltage_load_1")
print(voltage_load_1.to_c("speed_error_with_histeresis", "k_speed_i"))

print("voltage_load")
print(voltage_load.to_c("voltage_load_1", "dt"))

print("voltage_i")
print(voltage_i.to_c("voltage_i", "voltage_load"))

print("voltage_q")
print(voltage_q.to_c("voltage_p", "voltage_i"))

print("reference_voltage_q")
print( reference_voltage_q.to_c("voltage_q") )



