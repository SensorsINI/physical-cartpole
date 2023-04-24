"""
"double regression" (name is my invention) is a linear regression algorithm which fits two lines to two datasets,
while imposing that the two lines are required to have the same slope;
they are only allowed to have different intercept.
The proposed algorithm finds two such lines
which together minimize the total MSE of both datasets.

You can run this file as script to test the algorithm on a simple example.
"""

import numpy as np
import matplotlib.pyplot as plt

def double_regression(x1, y1, x2, y2):
    """
    Fittes two lines with MSE, one to x1,y1 and one to x2,y2
    Under an assumption they are required to have the same slope
    """

    n1 = len(x1)
    n2 = len(x2)

    sum_x1 = np.sum(x1)
    sum_y1 = np.sum(y1)

    sum_x2 = np.sum(x2)
    sum_y2 = np.sum(y2)

    avg_x1 = sum_x1/n1
    avg_y1 = sum_y1/n1

    avg_x2 = sum_x2/n2
    avg_y2 = sum_y2/n2

    xx = np.sum(x1**2) + np.sum(x2**2)
    xy =  np.dot(x1, y1) + np.dot(x2, y2)

    a = (xy - avg_y1*sum_x1 - avg_y2*sum_x2)/(xx - avg_x1*sum_x1 - avg_x2*sum_x2)
    b1 = avg_y1 - a*avg_x1
    b2 = avg_y2 - a*avg_x2

    return a, b1, b2


def double_regression_2(x1, y1, x2, y2):
    x2 = -x2
    y2 = -y2

    x = np.concatenate((x1, x2))
    y = np.concatenate((y1, y2))

    # number of observations/points
    n = np.size(x)

    # mean of x and y vector
    m_x = np.mean(x)
    m_y = np.mean(y)

    # calculating cross-deviation and deviation about x
    SS_xy = np.sum(y * x) - n * m_y * m_x
    SS_xx = np.sum(x * x) - n * m_x * m_x

    # calculating regression coefficients
    a = SS_xy / SS_xx
    b = m_y - a * m_x

    return a, b, -b




if __name__ == '__main__':
    A = 5.234
    B1 = 40.7
    B2 = B1
    x1 = np.linspace(-20, -5, 10)
    y1 = (A*x1 + B1)*(1+0.2*np.random.normal(size=x1.size))

    x2 = np.linspace(10, 30, 5)
    y2 = (A*x2 - B2)*(1+0.2*np.random.normal(size=x2.size))

    a, b1, b2 = double_regression_2(x1, y1, x2, y2)

    print('Found coefficients:')
    print('a:  {}'.format(a))
    print('b1: {}'.format(b1))
    print('b2: {}'.format(b2))

    plt.figure()
    plt.scatter(x1, y1)
    plt.plot(x1, a*x1+b1)
    plt.scatter(x2, y2)
    plt.plot(x2, a * x2 + b2)
    plt.show()