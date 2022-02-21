# Load relevant libraries
import numpy as np
import numba as nb
import matplotlib.pyplot as plt


# Goal is to implement a numba compatible polyfit (note does not include error handling)

# Define Functions Using Numba
# Idea here is to solve ax = b, using least squares, where a represents our coefficients e.g. x**2, x, constants
@nb.njit
def _coeff_mat(x, deg):
    mat_ = np.zeros(shape=(x.shape[0], deg + 1))
    const = np.ones_like(x)
    mat_[:, 0] = const
    mat_[:, 1] = x
    if deg > 1:
        for n in range(2, deg + 1):
            mat_[:, n] = x ** n
    return mat_


@nb.jit
def _fit_x(a, b):
    # linalg solves ax = b
    det_ = np.linalg.lstsq(a, b)[0]
    return det_


@nb.jit
def fit_poly(x, y, deg):
    a = _coeff_mat(x, deg)
    p = _fit_x(a, y)
    # Reverse order so p[0] is coefficient of highest order
    return p[::-1]


@nb.jit
def eval_polynomial(P, x):
    '''
    Compute polynomial P(x) where P is a vector of coefficients, highest
    order coefficient at P[0].  Uses Horner's Method.
    '''
    result = 0
    for coeff in P:
        result = x * result + coeff
    return result


# References
# 1. Numpy least squares docs -> https://docs.scipy.org/doc/numpy/reference/generated/numpy.linalg.lstsq.html#numpy.linalg.lstsq
# 2. Numba linear alegbra supported funcs -> https://numba.pydata.org/numba-doc/dev/reference/numpysupported.html#linear-algebra
# 3. SO Post -> https://stackoverflow.com/questions/56181712/fitting-a-quadratic-function-in-python-without-numpy-polyfit