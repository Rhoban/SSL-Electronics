#!/usr/bin/python3

from integer_calculus import *

#set_debug_mode( configure_pid_coe_to_max = True )

digits = 32

# Configure speed control
minimal_speed = -20.0
maximal_speed = 20.0

minimal_theta_error=-10.0
maximal_theta_error=10.0

maximal_k_pos_p = max(abs(maximal_speed), abs(minimal_speed)) / max(abs(maximal_theta_error),abs(minimal_theta_error))

# Configure Voltage control
reference_voltage = 1024

minimal_speed_error = -40
maximal_speed_error = 40

maximal_k_speed_p = reference_voltage / max(abs(maximal_speed_error),abs(minimal_speed_error))

print("--------------")
print("Evaluation of calculus errors")
print("--------------")

k_pos_p = Variable( minimal=0.0, maximal=maximal_k_pos_p, error=None, digits=digits, name="k_pos_p")
print( "k_pos_p : " + str( k_pos_p.final_error() ) )
k_pos_i = Variable( minimal=0.0, maximal=0.1, error=None, digits=digits, name="k_pos_i")
print( "k_pos_i : " + str( k_pos_i.final_error() ) )
#k_pos_d = Variable( minimal=0.1, maximal=10.0, error=None, digits=digits, name="k_pos_d")
#print( "k_pos_d : " + str( k_pos_d.final_error() ) )

k_speed_p = Variable(
    minimal=0.0, maximal=maximal_k_speed_p,
    error=None, digits=digits, name="k_speed_p"
)
print( "k_speed_p : " + str( k_speed_p.final_error() ) )
k_speed_i = Variable( minimal=0.0, maximal=0.1, error=None, digits=digits, name="k_speed_i")
print( "k_speed_i : " + str( k_speed_i.final_error() ) )
#k_speed_d = Variable( minimal=0.1, maximal=10.0, error=None, digits=digits, name="k_speed_d")
#print( "k_speed_d : " + str( k_speed_d.final_error() ) )

k_fem = Variable(
    minimal=0.0,
    maximal=reference_voltage/max(abs(maximal_speed), abs(minimal_speed)),
    error=None, digits=digits, name="k_fem"
)

dt = Constant(constant=1.0/3000.0, error=None, digits=digits)
print( "dt : " + str( dt.final_error() ) )
inv_dt = Constant(constant=3000.0, error=None, digits=digits)
print( "inv_dt : " + str( inv_dt.final_error() ) )

theta = Input( minimal=-6000.0, maximal=6000.0, scale=14, digits=digits, name="theta")
print( "theta : " + str( theta.final_error() ) )
theta_c = Variable( minimal=-6000.0, maximal=6000.0, error=None, digits=digits, name="theta_c")
print( "theta_c : " + str( theta_c.final_error() ) )

neg_theta_c = Neg(theta_c)
print( "neg_theta_c : " + str( neg_theta_c.final_error() ) )

theta_error = Add(
    term_1=theta, term_2=neg_theta_c, error=None, digits=digits,
    name="theta_error"
);
print( "theta_error : " + str( theta_error.final_error() ) )


limited_theta_error = Limit( 
    term = theta_error,
    minimal = minimal_theta_error,
    maximal = maximal_theta_error,
    name="limited_theta_error"
)
print( "limited_theta_error : " + str( limited_theta_error.final_error() ) )

speed_p = Mult(
    term_1=limited_theta_error, term_2=k_pos_p, error=None, digits=digits,
    name="speed_p"
)
#print( "t_min : " + str( speed_p.t_min/2**speed_p.scale ) )
#print( "max_error : " + str( speed_p.max_error/2**speed_p.scale ) )
#print( "k : " + str( speed_p.k ) )
print( "speed_p : " + str( speed_p.final_error() ) )

speed_load_1 = Mult(
    term_1=limited_theta_error, 
    term_2=k_pos_i, 
    error=None, digits=digits
)
print( "speed_load_1 : " + str( speed_load_1.final_error() ) )
speed_load = Mult(
    term_1 = speed_load_1,
    term_2 = dt,
    error=None, digits=digits,
    name="speed_load"
)
print( "speed_load : " + str( speed_load.final_error() ) )
speed_i = Accumulator(
    load=speed_load, minimal=minimal_speed, maximal=maximal_speed,
    digits=digits,
    name = "speed_i"
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
    error=None, digits=digits,
    name="speed_c"
)
print( "speed_c : " + str(speed_c.final_error()) )

speed = Input( minimal=-50.0, maximal=50.0, scale=14, digits=digits, name="speed" )
print( "speed : " + str( speed.final_error() ) )

neg_speed_c = Neg(speed_c)
print( "neg_speed_c : " + str( neg_speed_c.final_error() ) )

speed_error = Add(
    term_1=speed,
    term_2=neg_speed_c,
    error=None, digits=digits,
    name = "speed_error"
)
print( "speed_error : " + str( speed_error.final_error() ) )

limited_speed_error = Limit(
    term = speed_error,
    minimal = minimal_speed_error,
    maximal = maximal_speed_error,
    name = "limited_speed_error"
)
print( "limited_speed_error : " + str( limited_speed_error.final_error() ) )

voltage_p = Mult(
    term_1 = limited_speed_error,
    term_2 = k_speed_p,
    error = None,
    digits = digits,
    name = "voltage_p"
)
print( "voltage_p error : " + str( voltage_p.final_error() ) )


voltage_load_1 = Mult(
    term_1=limited_speed_error, term_2=k_speed_i, 
    error=None, digits=digits
)
print( "voltage_load_1 error : " + str( voltage_load_1.final_error() ) )
voltage_load = Mult(
    term_1 = voltage_load_1,
    term_2 = dt,
    error=None, digits=digits,
    name="voltage_load"
)

minimal_voltage = -reference_voltage
maximal_voltage = reference_voltage
print( "voltage_load error : " + str( voltage_load.final_error() ) )
voltage_i = Accumulator(
    load=voltage_load,
    minimal=minimal_voltage, maximal=maximal_voltage,
    digits=digits,
    name = "voltage_i"
)
print( "voltage_i error : " + str( voltage_i.final_error() ) )

electromagnetic_force = Mult(
    term_1=speed, term_2=k_fem, error=None, digits=digits,
    name="electromagnetic_force"
)
print( "electromagnetic_force error : " + str( electromagnetic_force.final_error() ) )

voltage_q = Add(
    term_1 = voltage_p,
    term_2 = Add(
        term_1=voltage_i,
        term_2=electromagnetic_force,
        error=None,
        digits = digits
    ),
    error = None,
    digits = digits,
    name = "voltage_q"
)
print( "voltage_q error : " + str( voltage_q.final_error() ) )

reference_voltage_q = Limit(
    term = voltage_q,
    minimal = -reference_voltage,
    maximal = reference_voltage,
    name="reference_voltage_q"
)
print( "limited_voltage_q error : " + str( voltage_q.final_error() ) )

#reference_voltage_q = Scale(
#    term=voltage_q, minimal=-reference_voltage, maximal=reference_voltage, 
#    digits=digits, name="reference_voltage_q"
#)
#print( "reference_voltage_q error : " + str( reference_voltage_q.final_error() ) )
assert( reference_voltage_q.final_error()< 0.5 );

#print("===========================")
#print("Complete calculus")
#print("===========================")
#
#print("k_pos_p")
#print(k_pos_p.to_c('i_k_pos_p'))
#
#print("k_pos_i")
#print(k_pos_i.to_c('i_k_pos_i'))
#
#print("k_speed_p")
#print(k_speed_p.to_c('i_k_speed_p'))
#
#print("k_speed_i")
#print(k_speed_i.to_c('i_k_speed_i'))
#
#print("dt")
#print(dt.to_c())
#
#print("inv_dt")
#print(inv_dt.to_c())
#
#print("theta")
#print(theta.to_c('theta'))
#
#print("theta_c")
#print(theta_c.to_c('i_theta_c'))
#
#print("neg_theta_c")
#print(neg_theta_c.to_c('theta_c'))
#
#print("theta_error")
#print(theta_error.to_c('theta', 'neg_theta_c'))
#
#print("limited_theta_error")
#print(limited_theta_error.to_c('theta_error'))
#
#print("speed_p")
#print(speed_p.to_c('limited_theta_error', 'k_pos_p' ))
#
#print("speed_load_1")
#print(speed_load_1.to_c('limited_theta_error', 'k_pos_i'))
#
#print("speed_load")
#print(speed_load.to_c('speed_load_1', 'dt'))
#
#print("speed_i")
#print(speed_i.to_c('speed_i', 'speed_load'))
#
#print("speed_c")
#print(speed_c.to_c('speed_p', 'speed_i'))
#
#print("speed")
#print(speed.to_c("speed"))
#
#print("neg_speed_c")
#print(neg_speed_c.to_c("speed_c"))
#
#print("speed_error")
#print(speed_error.to_c("speed", "neg_speed_c"))
#
#print("limited_speed_error")
#print(limited_speed_error.to_c("speed_error"))
#
#print("voltage_p")
#print(voltage_p.to_c("limited_speed_error", "k_speed_p"))
#
#print("voltage_load_1")
#print(voltage_load_1.to_c("limited_speed_error", "k_speed_i"))
#
#print("voltage_load")
#print(voltage_load.to_c("voltage_load_1", "dt"))
#
#print("voltage_i")
#print(voltage_i.to_c("voltage_i", "voltage_load"))
#
#print("voltage_q")
#print(voltage_q.to_c("voltage_p", "voltage_i"))
#
#print("reference_voltage_q")
#print( reference_voltage_q.to_c("voltage_q") )

print("===========================")
print("Complete program")
print("===========================")

print( reference_voltage_q.prog() )

