#coding=utf-8

# Copyright (C) 2019  Adrien Boussicault <adrien.boussicault@labri.fr>
#
# This program is free software: you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation, either
# version 3 of the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public
# License along with this program.  If not, see
# <http://www.gnu.org/licenses/>.

#from math import *
#import scipy.signal
#import matplotlib.pyplot as plt

# Motor frequence = angular velocity in turn . s^-1
PI = float(pi)

fm = 50.0  # Hz
wfm = 2*PI*fm
wfm_2 = 2*wfm

# Sample frequence
fe = 10000.0  # Hz
# Sample period
Te = 1.0/fe #Hz
we = 2*PI*fe
# Pulsation de filtrage numérique et analogique 
# =============================================
#
# Maximal analogic frequence
#---------------------------
#
# wn : pulsaion de filtrage numérique
# wn < 2.Pi/(N.Te)  # Critère de Niquist Shannon ( fn < fe/2 )
# N >= 2
#
# We choos in pratic 
# N >= 10
#
# wa : pulsation de filtrage analogique
# wa = 2/Te . tan( Wn . Te / 2 )   si Wn << 1/Te
#
# Ainsi, 
#
# wa < 2/Te . tan(Pi/N)
# 
#Pulsation de filtrage
N = 10.0
max_wa = 2.0/Te * tan(PI/N)
print( "max_wa :" +  str(max_wa) )


#
# Maximal frequence
# -----------------
M = 4.0
wa = 2*PI*fm*M
print( "wa : " + str(wa) )
assert( wa < max_wa )

def butterworth(s, g, wc, n=1):
    v = s/wc
    if n==1 :
        return g/(v+1.0)
    if n==2:
        return g/(v**2 + sqrt(2.0)*v + 1.0)
    if n==3:
        return g/((v+1.0)*(v**2+v+1.0))
    if n==4:
        return g/((v**2+0.7654*v+1.0)*(v**2+1.8478*v+1.0))
    if n==5:
        return g/((v+1.0)*(v**2+0.6180*v+1.0)*(v**2+1.618*v+1.0))

def z_filter_butterworth(z, g, wc, te, n=1):
    return butterworth( (2.0/te)*(z-1)/(z+1), g, wc, n )

def z_coef_butterworth(g, wc, te, n=1):
    var('z')
    f = z_filter_butterworth(z, g, wc, Te, n)
    f_num = expand( numerator( f ) )
    f_den = expand( denominator( f ) )
    return [
        list(reversed( f_num.coefficients(sparse=False) )),
        list(reversed( f_den.coefficients(sparse=False) ))
    ]

def rat_butterworth(g, wc, te, n=1):
    var('z')
    f = z_filter_butterworth(z, g, wc, Te, n)
    f_num = expand( numerator( f ) )
    f_den = expand( denominator( f ) )
    return f_num/f_den

def code_rat_butterworth(g, wc, te, n=1):
    f = z_coef_butterworth(g, wc, te, n)
    return filtrer_code( f )

def filtrer( coefs, signal ):
    size = len(signal)
    num = coefs[0]
    den = coefs[1]
    assert( len(den) == len(num) )
    n = len(num)
    y = [0.0 for i in range(n-1)]
    for i in range(0, size):
        y.append(
            (1/den[0])*(
                sum( [signal[i-j]*num[j] for j in range(n) ] ) -
                sum( [y[-j]*den[j] for j in range(1,n)] )
            )
        )
    res = y[n-1:]
    return y[n-1:]


def filtrer_code( coefs ):
    num = coefs[0]
    den = coefs[1]
    assert( len(den) == len(num) )
    n = len(num)
    res = ""
    res += 'y_0 = '
    res += '('
    res += '('
    for j in range(n):
        res += ( str(num[j]) + '* s_' + str(j) + ' + ' )
    res += ')/' + str(den[0]) + ' + '
    for j in range(n-1):
        res += ( str(-den[j+1]/den[0]) + '* y_' + str(j) + ' + ' )
    res += ')'
    return res


def filtrer_average( signal, n ):
    y = [0.0 for i in range(n-1)]
    for i in range(n-1, len( signal)):
        y.append(
            (1.0/n)*(
                sum( [signal[i-j] for j in range(n) ] )
            )
        )
    return y


def module(w, g, wc, n):
    return abs( butterworth(w*I,g,wc,n) )

def phase(w, g, wc, n):
    return arg( butterworth(w*I,g,wc,n) )

def complete_phase(w, g, wc, n):
    p = float(phase(1,g,wc,n))
    cnt = 0
    v = 0
    while(v<w):
      v += 10
      p_old = p 
      p = float(phase(v,g,wc,n))
      if abs(p - p_old) > 2.0 :
        if p > p_old:
          cnt += 1
        else:
          cnt -= 1
    return float( phase(w,g,wc,n) - 2*pi*cnt )

def phase_360(w, g, wc, n):
    val = phase(w, g, wc, n)
    while( val >= 0 ):
        val -= 2*pi
    return val*360/(2*pi)

def show():
    nb = 300
    xmax = 8000
    xmin = 0
    p = plot(module(x, 1.0, wa, 1), (x, xmin, xmax), ymax=1.0, ymin=0.0, xmin=xmin, xmax=xmax, plot_points=nb, color='red', adaptive_recursion=7)
    p += plot(module(x, 1.0, wa, 2), (x, xmin, xmax), ymax=1.0, ymin=0.0, xmin=xmin, xmax=xmax, plot_points=nb, color='blue', adaptive_recursion=7 )
    p += plot(module(x, 1.0, wa, 3), (x, xmin, xmax), ymax=1.0, ymin=0.0, xmin=xmin, xmax=xmax, plot_points=nb, color='green', adaptive_recursion=7 )
    p += plot(module(x, 1.0, wa, 4), (x, xmin, xmax), ymax=1.0, ymin=0.0, xmin=xmin, xmax=xmax, plot_points=nb, color='grey', adaptive_recursion=7 )
    p += plot(module(x, 1.0, wa, 5), (x, xmin, xmax), ymax=1.0, ymin=0.0, xmin=xmin, xmax=xmax, plot_points=nb, color='pink', adaptive_recursion=7 )
    p += line([(wfm,0), (wfm,1)],linestyle=":")
    p += line([(wfm_2,0), (wfm_2,1)],linestyle=":")
    p += line([(max_wa,0), (max_wa,1)],linestyle=":")
    p.show()


def show_phase():
    nb = 300
    xmax = 3000
    xmin = 0
    ymax=-400
    ymin=-400
    p = plot(lambda x:phase_360(x, 1.0, wa, 1), (x, xmin, xmax), xmin=xmin, xmax=xmax, plot_points=nb, color='red', adaptive_recursion=7)
    p += plot(lambda x:phase_360(x, 1.0, wa, 2), (x, xmin, xmax), xmin=xmin, xmax=xmax, plot_points=nb, color='blue', adaptive_recursion=7 )
    p += plot(lambda x:phase_360(x, 1.0, wa, 3), (x, xmin, xmax), xmin=xmin, xmax=xmax, plot_points=nb, color='green', adaptive_recursion=7 )
    p += plot(lambda x:phase_360(x, 1.0, wa, 4), (x, xmin, xmax), xmin=xmin, xmax=xmax, plot_points=nb, color='grey', adaptive_recursion=7 )
    p += plot(lambda x:phase_360(x, 1.0, wa, 5), (x, xmin, xmax), xmin=xmin, xmax=xmax, plot_points=nb, color='pink', adaptive_recursion=7 )
    p += line([(wfm,0), (wfm,-360)],linestyle=":")
    p += line([(wfm_2,0), (wfm_2,-360)],linestyle=":")
    p += line([(max_wa,0), (max_wa,-360)],linestyle=":")
    p.show()


class Bruit:
    def __init__(self,w, N):
        self.N = 10
        self.w = w
        self.bruits = [
            [ .1*random(), 10*wfm*(1+i* random()), 2*PI*random() ]
            for i in range(self.N)
        ]

    def __call__(self, t):
        res = self.sans_bruit(t)
        #for [coef, pulsation, phase] in self.bruits:
        #    res += coef*sin(pulsation*t+phase)
        #res += (random()-.5)*.1
        return res

class sin_bruit( Bruit ):
    def __init__(self, w, N=20):
        Bruit.__init__(self, w, N)

    def sans_bruit(self, t):
        if(t<0.06):
            return 0.0
        return sin(self.w*(t-0.06))
        #return sin(self.w*(t-0.06))*(t-0.06)*(25.0/4.0)
        #return (t-0.06)*(25.0/4.0)

class droite_bruit( Bruit ):
    def __init__(self, w, N=20):
        Bruit.__init__(self, w, N)

    def sans_bruit(self, t):
        if(t<0.06):
            return 0.0
        return (t-0.06)*(25.0/4.0)

class rampe_bruit( Bruit ):
    def __init__(self, w, N=20):
        Bruit.__init__(self, w, N)

    def sans_bruit(self, t):
        if(t<0.06):
            return 0.0
        return 1

class exp_bruit( Bruit ):
    def __init__(self, w, N=20):
        Bruit.__init__(self, w, N)

    def sans_bruit(self, t):
        if(t<0.06):
            return 0.0
        return 1 - exp( -100*(t-0.06) )

N = 3
G = 1.0
coefs = z_coef_butterworth(G, wa, Te, n=N)
sin_b = sin_bruit( 2*PI*6.0 )
# sin_b = sin_bruit( 2*PI*2000.0 )
rampe_b = rampe_bruit( 2*PI*25.0 )
droite_b = droite_bruit( 2*PI*25.0 )
exp_b = exp_bruit( 2*PI*25.0, 20 )
time = [ i*Te for i in range(int((4/25.0)*fe)) ]
#signal = [ droite_b(t) for t in time ]
signal = [ sin_b(t) for t in time ]
#signal_average = filtrer_average( signal, 5 )
signal_filtre = filtrer( coefs, signal )
#signal_filtre = filtrer_average( signal, 5 )
assert( len(signal) == len(signal_filtre) )

def show_sin_filtre():
    nb = 20
    xmin = 0.0
    xmax = 4/25.0
    points_signal = [ (time[i], signal[i]) for i in range(len(time)) ]
    points_signal_filtre = [ (time[i], signal_filtre[i]) for i in range(len(time)) ]
    N_E = 50
    points_signal_filtre_sous_ech = [ (time[N_E*i] - phase_delay(wfm,G,wa,N), signal_filtre[N_E*i]) for i in range(len(time)/N_E) ]
    p = plot(droite_b.sans_bruit(x), (x, 0.06, xmax), xmin=xmin, xmax=xmax, plot_points=nb, color='red', adaptive_recursion=7 )
    for pt in points_signal:
        p+= point(pt)
    for pt in points_signal_filtre:
        p+= point(pt,color='pink')
    for pt in points_signal_filtre_sous_ech:
        p+= point(pt,color='green')
    #p += plot(exp_b.sans_bruit(x), (x, 0.06, xmax), xmin=xmin, xmax=xmax, plot_points=nb, color='black', adaptive_recursion=7 )
    p.show()

def phase_delay(w, g, wc, n):
    return -float(phase(w, G, wa, i) )/w


for i in range( 1, 5 ):
    print("---------------------")
    print("Butter worth N = " + str(i))
    #print( z_coef_butterworth(G, wa, Te, n=i) )
    print( z_coef_butterworth(G, wa, Te, n=i))
    print( rat_butterworth(G, wa, Te, n=i) )
    print(code_rat_butterworth(G, wa, Te, n=i))
    print( "decalage temporeltemporel  : " + str( phase_delay(wfm,G,wa,i)))



