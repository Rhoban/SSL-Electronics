#!/usr/bin/python3

from asserv import *

sin_output_max = 13
c1 = Input( minimal=-1.0, maximal=1.0, scale=sin_output_max, digits=digits, name="c1")
s1 = Input( minimal=-1.0, maximal=1.0, scale=sin_output_max, digits=digits, name="s1")
c2 = Input( minimal=-1.0, maximal=1.0, scale=sin_output_max, digits=digits, name="c2")
s2 = Input( minimal=-1.0, maximal=1.0, scale=sin_output_max, digits=digits, name="s2")
c3 = Input( minimal=-1.0, maximal=1.0, scale=sin_output_max, digits=digits, name="c3")
s3 = Input( minimal=-1.0, maximal=1.0, scale=sin_output_max, digits=digits, name="s3")

direct_voltage_c = Input(
    minimal=-reference_voltage,
    maximal=reference_voltage, 
    scale=0,
    digits=digits, name="direct_voltage_c"
)
quadrature_voltage_c = Input(
    minimal=-reference_voltage,
    maximal=reference_voltage, 
    scale=0,
    digits=digits,
    name="quadrature_voltage_c"
)

phase_voltage_u = TrueLimits(
    term=Add(
        term_1=Mult(
            term_1=c1,
            term_2=direct_voltage_c,
            error=None, digits=digits
        ),
        term_2=Neg(
            Mult(
                term_1=s1,
                term_2=quadrature_voltage_c,
                error=None, digits=digits
            )
        ),
        error=None, digits=digits,
        name="phase_voltage_u"
    ),
    minimal = -reference_voltage,
    maximal = reference_voltage
)

phase_voltage_v = TrueLimits(
    term = Add(
        term_1=Mult(
            term_1=c2,
            term_2=direct_voltage_c,
            error=None, digits=digits
        ),
        term_2=Neg(
            Mult(
                term_1=s2,
                term_2=quadrature_voltage_c,
                error=None, digits=digits
            )
        ),
        error=None, digits=digits,
        name="phase_voltage_v"
    ),
    minimal = -reference_voltage,
    maximal = reference_voltage
)

phase_voltage_w =  TrueLimits(
    term = Add(
        term_1=Mult(
            term_1=c3,
            term_2=direct_voltage_c,
            error=None, digits=digits
        ),
        term_2=Neg(
            Mult(
                term_1=s3,
                term_2=quadrature_voltage_c,
                error=None, digits=digits
            )
        ),
        error=None, digits=digits,
        name="phase_voltage_w"
    ),
    minimal = -reference_voltage,
    maximal = reference_voltage
)

min_voltage = Min(
    term_1 = Min(
        term_1=phase_voltage_u,
        term_2=phase_voltage_v,
        digits=digits
    ),
    term_2=phase_voltage_w,
    digits=digits,
    name='min_voltage'
)

user_pwm = Variable(
    minimal=-100,
    maximal=100,
    error=None,
    digits=digits, name="user_pwm"
)

OVERLAP_PWM = 0.0 # 2.0
PWM_SUP = 3000.0
USER_PWM_SUP = 100
INV_ALPHA = 3.0*reference_voltage
ALPHA_PWM = (PWM_SUP-OVERLAP_PWM)/(USER_PWM_SUP*INV_ALPHA)

alpha_pwm = Constant(
    constant=ALPHA_PWM, error=None, digits=digits, name = "alpha_pwm"
)

from math import sqrt
max_phase_difference = sqrt(3)*reference_voltage

alpha_user_pwm = Mult(
    term_1 = alpha_pwm,
    term_2 = user_pwm,
    error=None, digits=digits,
    name = "alpha_user_pwm"
)

phase_pwm_u = Rescale(
    term = Mult(
        term_1 = alpha_user_pwm,
        term_2 = TrueLimits( # We know the bounds of the difference voltages.
            term=Add(
                term_1=phase_voltage_u,
                term_2=Neg(min_voltage),
                error=None, digits=digits
            ),
            minimal = -max_phase_difference,
            maximal = max_phase_difference
        ),
        error=None, digits=digits
    ),
    scale = 0,
    digits = digits,
    name="phase_pwm_u"
)

phase_pwm_v = Rescale(
    term = Mult(
        term_1 = alpha_user_pwm,
        term_2 = TrueLimits( # We know the bounds of the difference voltages.
            term=Add(
                term_1=phase_voltage_v,
                term_2=Neg(min_voltage),
                error=None, digits=digits
            ),
            minimal = -max_phase_difference,
            maximal = max_phase_difference
        ),
        error=None, digits=digits
    ),
    scale = 0,
    digits = digits,
    name="phase_pwm_v"
)

phase_pwm_w = Rescale(
    term = Mult(
        term_1 = alpha_user_pwm,
        term_2 = TrueLimits( # We know the bounds of the difference voltages.
            term=Add(
                term_1=phase_voltage_w,
                term_2=Neg(min_voltage),
                error=None, digits=digits
            ),
            minimal = -max_phase_difference,
            maximal = max_phase_difference
        ),
        error=None, digits=digits
    ),
    scale = 0,
    digits = digits,
    name="phase_pwm_w"
)

if __name__=="__main__":
    print( phase_pwm_u )
    print( phase_pwm_v )
    print( phase_pwm_w )

    print("")
    print("")
    print("###############################################################")
    print("###############################################################")
    print("##  phase_pwm_u  ##############################################")
    print("###############################################################")
    print("")

    print( phase_pwm_u.prog() )

    print("")
    print("")
    print("###############################################################")
    print("###############################################################")
    print("##  phase_pwm_v  ##############################################")
    print("###############################################################")
    print("")

    print( phase_pwm_v.prog() )

    print("")
    print("")
    print("###############################################################")
    print("###############################################################")
    print("##  phase_pwm_w  ##############################################")
    print("###############################################################")
    print("")

    print( phase_pwm_w.prog() )
