#!/usr/bin/python3

from integer_calculus import *

#set_debug_mode( configure_pid_coe_to_max = True )

motor_frequence = 800.0
motor_dt = 1/motor_frequence

digits = 32

# Configure speed control
minimal_speed = -10.0
maximal_speed = 10.0

minimal_theta_error=-.9
maximal_theta_error=.9

k_pos_p_max = 4*max(abs(maximal_speed), abs(minimal_speed)) / max(abs(maximal_theta_error),abs(minimal_theta_error))
temps_reactivite = 1/100.0
k_pos_i_max = max(abs(maximal_speed), abs(minimal_speed))/(
    temps_reactivite*max(abs(maximal_theta_error),abs(minimal_theta_error))
)

# Configure Voltage control

nb_bits_refernece_voltage = 10
reference_voltage = 2**nb_bits_refernece_voltage

minimal_speed_error = -2.4
maximal_speed_error = 2.4

maximal_k_speed_p = 2*reference_voltage / max(abs(maximal_speed_error),abs(minimal_speed_error))

k_pos_p = Variable( minimal=0.0, maximal=k_pos_p_max, error=None, digits=digits, name="k_pos_p")
k_pos_i = Variable( minimal=0.0, maximal=k_pos_i_max, error=None, digits=digits, name="k_pos_i")
#k_pos_d = Variable( minimal=0.1, maximal=10.0, error=None, digits=digits, name="k_pos_d")

k_speed_p = Variable(
    minimal=0.0, maximal=maximal_k_speed_p,
    error=None, digits=digits, name="k_speed_p"
)

temps_reactivite = 1/100.0
k_speed_i_max = reference_voltage/(
    temps_reactivite*max(abs(maximal_speed_error),abs(minimal_speed_error))
)
k_speed_i = Variable( minimal=0.0, maximal=k_speed_i_max, error=None, digits=digits, name="k_speed_i")
#k_speed_d = Variable( minimal=0.1, maximal=10.0, error=None, digits=digits, name="k_speed_d")

k_fem_max = 2.0 * reference_voltage/max(abs(maximal_speed), abs(minimal_speed))
k_fem = Variable(
    minimal=0.0,
    maximal=k_fem_max,
    error=None, digits=digits, name="k_fem"
)

dt = Constant(constant=1.0/motor_frequence, error=None, digits=digits)
inv_dt = Constant(constant=motor_frequence, error=None, digits=digits)

total_time_match = 20 * 60 * 1.0
maximal_number_of_turns=maximal_speed * total_time_match
theta = Input(
    minimal=-maximal_number_of_turns, maximal=maximal_number_of_turns,
    scale=14, digits=digits, name="theta"
)
theta_c = Variable(
    minimal=-maximal_number_of_turns, maximal=maximal_number_of_turns,
    error=None, digits=digits, name="theta_c"
)

neg_theta = Neg(theta)

theta_error = Add(
    term_1=theta_c, term_2=neg_theta, error=None, digits=digits,
    name="theta_error"
);


limited_theta_error = Limit( 
    term = theta_error,
    minimal = minimal_theta_error,
    maximal = maximal_theta_error,
    name="limited_theta_error"
)

speed_p = Mult(
    term_1=limited_theta_error, term_2=k_pos_p, error=None, digits=digits,
    name="speed_p"
)

speed_load_1 = Mult(
    term_1=limited_theta_error, 
    term_2=k_pos_i, 
    error=None, digits=digits
)
speed_load = Mult(
    term_1 = speed_load_1,
    term_2 = dt,
    error=None, digits=digits,
    name="speed_load"
)
speed_i = Accumulator(
    load=speed_load, minimal=minimal_speed, maximal=maximal_speed,
    digits=digits,
    name = "speed_i"
)

#speed_d = Mult(
#    Mult( term_1=theta_error, term_2=k_pos_d, error=None, digits=digits ),
#    inv_dt, error=None, digits=digits 
#)

speed_c = Limit(
    term = Add(
        term_1 = speed_p,
        term_2 = speed_i,
        error=None, digits=digits,
    ),
    minimal = minimal_speed,
    maximal = maximal_speed,
    name = "speed_c"
)

speed_csg = Rescale(
    term=Variable(
        minimal=speed_c.minimal, maximal=speed_c.maximal, 
        error=None,
        digits=digits,
        name = "speed_csg"
    ), scale = speed_c.scale,
    digits = digits,
    name="speed_c"
)

SPEED_NORMALISATION=20
speed = Input( minimal=-32.0, maximal=32.0, scale=SPEED_NORMALISATION, digits=digits, name="speed" )

neg_speed = Neg(speed)

speed_error = Add(
    term_1=speed_c,
    term_2=neg_speed,
    error=None, digits=digits,
    name = "speed_error"
)

limited_speed_error = Limit(
    term = speed_error,
    minimal = minimal_speed_error,
    maximal = maximal_speed_error,
    name = "limited_speed_error"
)

voltage_p = Mult(
    term_1 = limited_speed_error,
    term_2 = k_speed_p,
    error = None,
    digits = digits,
    name = "voltage_p"
)

voltage_load_1 = Mult(
    term_1=limited_speed_error, term_2=k_speed_i, 
    error=None, digits=digits
)
voltage_load = Mult(
    term_1 = voltage_load_1,
    term_2 = dt,
    error=None, digits=digits,
    name="voltage_load"
)

minimal_voltage = -reference_voltage
maximal_voltage = reference_voltage
voltage_i = Accumulator(
    load=voltage_load,
    minimal=minimal_voltage, maximal=maximal_voltage,
    digits=digits,
    name = "voltage_i"
)

electromagnetic_force = Mult(
    term_1=speed, term_2=k_fem, error=None, digits=digits,
    name="electromagnetic_force"
)

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

reference_voltage_q = Limit(
    term = voltage_q,
    minimal = -reference_voltage,
    maximal = reference_voltage,
    name="reference_voltage_q"
)

output_voltage_q = Rescale(
    term=reference_voltage_q,
    scale = 0,
    digits = digits,
    name="output_voltage_q"
)


theta_csg = Rescale(
    term=Input(
        minimal=theta.minimal, maximal=theta.maximal, scale=theta.scale, 
        digits=digits, 
        name="theta_csg"
    ), scale = theta_c.scale,
    digits = digits,
    name="theta_c"
)

#reference_voltage_q = Scale(
#    term=voltage_q, minimal=-reference_voltage, maximal=reference_voltage, 
#    digits=digits, name="reference_voltage_q"
#)
if( not reference_voltage_q.final_error()< 6.0 ):
    raise ValueError(
        "Not enough precision : " + str(reference_voltage_q.final_error())
    )

if __name__=='__main__':
    print("===========================")
    print("Maximal errors")
    print("===========================")

    reference_voltage_q.print_errors()

    print("===========================")
    print("Complete program")
    print("===========================")

    print( output_voltage_q.prog() )

    print("===========================")
    print("Input to by pass speed_c with the consign speed_csg")
    print("===========================")
    print( speed_csg.prog() )

    print("===========================")
    print("Input to by pass speed_c with the consign speed_csg")
    print("===========================")
    print( theta_csg.prog() )
