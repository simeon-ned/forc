from numpy import linspace, arange, meshgrid

from matplotlib.pyplot import *

from scipy.integrate import odeint
from sympy import Matrix, symbols, solve


def simulate(f,
             init_state,
             t0=0,
             tf=1,
             N=500,
             size =(6, 4),
             show_plot=False):

    t = linspace(t0, tf, N)  # Create time span

    x_sol = []
    for x_init in init_state:
        # integrate system "sys_ode" from initial state $x0$
        x_sol.append(odeint(f, x_init, t))

    figure(figsize=size)
    if show_plot:
        for sol in x_sol:
            plot(t, sol, linewidth=2.0)
        grid(color='black', linestyle='--', linewidth=1.0, alpha=0.7)
        grid(True)
        xlim([t0, tf])
        ylabel(r'State $x$')
        xlabel(r'Time $t$ (s)')
        tight_layout()
    # show()
        show()


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
    x1_span = arange(-1.1*x1_max, 1.1*x1_max, 0.1)
    x2_span = arange(-1.1*x2_max, 1.1*x2_max, 0.1)
    x1_grid, x2_grid = meshgrid(x1_span, x2_span)
    dx1, dx2 = f([x1_grid, x2_grid], 0)

    dist = (x1_grid**2 + x2_grid**2)**0.5
    lw = 0.8*(2*dist + dist.max()) / dist.max()

    # figure(figsize=size)
    title('Phase Portrait')

    if contour:
        contourf(x1_span, x2_span, dist, cmap=cmap, alpha=0.15)

    streamplot(x1_span, x2_span, dx1, dx2, arrowsize=1.2,   density=density, color=dist,
               cmap=cmap, linewidth=lw, arrowstyle='->')  # ,color=L, cmap='autumn', linewidth = lw)

    xlabel(r'State  $x_1$')
    ylabel(r'State  $x_2$')

    xlim([-x1_max, x1_max])
    ylim([-x2_max, x2_max])
    if draw_grid:
        grid(color='black', linestyle='--', linewidth=1.0, alpha=0.3)
        grid(True)
    tight_layout()
    # show()

    return None


def phase_graph(f,
                x_range=[1, 1],
                size=(6, 4),
                # draw_grid=False,
                show_plot = True
                ):

    x = linspace(x_range[0], x_range[1], 100)
    dx = f(x, 0)
    if show_plot:
        figure(figsize=size)
        plot(x, dx, color='k')
        xlabel(r'State  $x$')
        ylabel(r'Derivative  $\dot{x}$')
        hlines(0, x_range[0], x_range[1], color='k', linestyle='--', alpha=0.6)
        xlim([x_range[0], x_range[1]])
        grid(color='black', linestyle='--', linewidth=1.0, alpha=0.3)
        grid(True)
        tight_layout()
        show()

    return dx


def symbolical_jacobian(f, x):
    # x = symbols(r'x_1, x_2')
    f_sym = Matrix([f(x)]).T
    jacobian = f_sym.jacobian(x)
    return jacobian
