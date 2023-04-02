import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint # import integrator routine


def simulate(f,
             init_state,
             t0=0,
             tf=1,
             N=500,
             size =(6, 4),
             show_plot=False):

    t = np.linspace(t0, tf, N)  # Create time span

    x_sol = []
    for x_init in init_state:
        # integrate system "sys_ode" from initial state $x0$
        x_sol.append(odeint(f, x_init, t))

    plt.figure(figsize=size)
    if show_plot:
        for sol in x_sol:
            plt.plot(t, sol, linewidth=2.0)
        plt.grid(color='black', linestyle='--', linewidth=1.0, alpha=0.7)
        plt.grid(True)
        plt.xlim([t0, tf])
        plt.ylabel(r'State $x$')
        plt.xlabel(r'Time $t$ (s)')
        plt.tight_layout()
    # show()
        plt.show()


    return x_sol


def phase_portrait(f,
                   x_range=[1, 1],
                   cmap='gray',
                   contour=False,
                   #    show_plot=False,
                   size=(7, 5),
                   density=0.95,
                   draw_grid=False,
                   ):

    x1_max, x2_max = x_range
    x1_span = np.arange(-1.1*x1_max, 1.1*x1_max, 0.1)
    x2_span = np.arange(-1.1*x2_max, 1.1*x2_max, 0.1)
    x1_grid, x2_grid = np.meshgrid(x1_span, x2_span)
    dx1, dx2 = f([x1_grid, x2_grid], 0)

    dist = (x1_grid**2 + x2_grid**2)**0.5
    lw = 0.8*(2*dist + dist.max()) / dist.max()

    # figure(figsize=size)
    plt.title('Phase Portrait')

    if contour:
        plt.contourf(x1_span, x2_span, dist, cmap=cmap, alpha=0.15)

    plt.streamplot(x1_span, x2_span, dx1, dx2, arrowsize=1.2,   density=density, color=dist,
               cmap=cmap, linewidth=lw, arrowstyle='->')  # ,color=L, cmap='autumn', linewidth = lw)

    plt.xlabel(r'State  $x_1$')
    plt.ylabel(r'State  $x_2$')

    plt.xlim([-x1_max, x1_max])
    plt.ylim([-x2_max, x2_max])
    if draw_grid:
        plt.grid(color='black', linestyle='--', linewidth=1.0, alpha=0.3)
        plt.grid(True)
    plt.tight_layout()
    # show()

    return None
