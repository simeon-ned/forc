import numpy as np
from matplotlib.pyplot import *
from scipy.integrate import odeint

def simulate(f, init_state, t0=0, tf=1, N = 500, show_plot=False):
    t = np.linspace(t0, tf, N) # Create time span

    x_sol = []    
    for x_init in init_state:
        x_sol.append(odeint(f, x_init, t))  # integrate system "sys_ode" from initial state $x0$

    if show_plot:
        for sol in x_sol:
            plot(t, sol, linewidth=2.0)
        grid(color='black', linestyle='--', linewidth=1.0, alpha = 0.7)     
        grid(True)
        xlim([t0, tf])
        ylabel(r'State $x$')
        xlabel(r'Time $t$ (s)')
        show()

    return x_sol 


