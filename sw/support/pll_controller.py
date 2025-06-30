import sympy

# PLL controller based on https://kiedontaa.blogspot.com/2024/07/single-parameter-controller-for-gps.html
# Slight modifications:
# - Integral of the phase error is computed from the observed error instead of the averaged error.
# - Take the eigenvalues of the matrix to be (1 - r) instead of r, to simplify control parameter expressions.

# OCXO model is given as
# e_{n+1} = e_n - dt * gain * ( u_n - u0 ), (1)
# where e_n is the phase error at time n, u_n is the control signal at time n.
# dt is the time step, gain is the OCXO gain, and u0 is a constant offset of the OCXO control signal.

# The phase error observation is noisy, and needs to be averaged over several samples.
# The averaged phase error is given as
# ehat_{n+1} = alpha * e_n + (1 - alpha) * ehat_n, (2)
# where alpha is the averaging factor.

# The integral of the averaged phase error is given as
# i_{n+1} = i_n + e_n (3)

# The PLL controller is given as
# u_n = P * ehat_n + I * i_n, (4)
# where P is the proportional gain and I is the integral gain.

# Plugging (4) into (1), we get
# e_{n+1} = e_n - dt * gain * ( P * ehat_n + I * i_n - u0 ) (5)

# Arranging everything in matrix form, we get the following system of equations:
# [    e_{n+1} ]   [     1, -dt * gain * P, -dt * gain * I ] [    e_n ]   [ dt * gain * u0 ]
# [ ehat_{n+1} ] = [ alpha,      1 - alpha,              0 ] [ ehat_n ] + [              0 ]
# [    i_{n+1} ]   [     1,              0,              1 ] [    i_n ]   [              0 ]

dt_gain, P, I, u0, alpha, r, lbd = sympy.symbols('dt_gain P I u0 alpha r lambda')

A = sympy.Matrix([
    [    1, -dt_gain * P, -dt_gain * I],
    [alpha,    1 - alpha,            0],
    [    1,            0,            1]
])

# Require that the eigenvalues of A all are (1-r)
# This is equivalent to the characteristic polynomial of A being equal to (lambda - (1-r))^3
target_poly = sympy.poly((lbd - (1 - r))**3, lbd)

sol = sympy.solve(A.charpoly(lbd) - target_poly, [P, I, alpha])
print(sol)
