"""Two-sided 95% Student-t critical value for df = n - 1.

No SciPy in this repository. The value is the 0.975 quantile of the t
distribution, from the regularized incomplete beta function.
"""

from __future__ import annotations

import math


def student_t_crit_95(df: int) -> float:
    """Return t such that P(|T_df| > t) = 0.05."""
    if isinstance(df, bool) or not isinstance(df, int) or df < 1:
        raise ValueError("degrees of freedom must be a positive integer")
    return _student_t_quantile(df, 0.975)


def _student_t_quantile(df: int, probability: float) -> float:
    if probability <= 0.5 or probability >= 1.0:
        raise ValueError("probability must be in (0.5, 1)")
    low = 0.0
    high = 1.0
    while _student_t_cdf(df, high) < probability:
        high *= 2.0
        if high > 1e6:
            raise ValueError("Student-t quantile failed to bracket")
    for _ in range(80):
        mid = 0.5 * (low + high)
        if _student_t_cdf(df, mid) < probability:
            low = mid
        else:
            high = mid
    return 0.5 * (low + high)


def _student_t_cdf(df: int, value: float) -> float:
    if value == 0.0:
        return 0.5
    x = df / (df + value * value)
    tail = 0.5 * _regularized_incomplete_beta(0.5 * df, 0.5, x)
    if value > 0.0:
        return 1.0 - tail
    return tail


def _regularized_incomplete_beta(a: float, b: float, x: float) -> float:
    if x <= 0.0:
        return 0.0
    if x >= 1.0:
        return 1.0
    log_beta = math.lgamma(a + b) - math.lgamma(a) - math.lgamma(b) + a * math.log(x) + b * math.log(1.0 - x)
    front = math.exp(log_beta)
    if x < (a + 1.0) / (a + b + 2.0):
        return front * _betacf(a, b, x) / a
    return 1.0 - front * _betacf(b, a, 1.0 - x) / b


def _betacf(a: float, b: float, x: float) -> float:
    """Lentz continued fraction for the incomplete beta function."""
    max_iter = 200
    epsilon = 3e-14
    qab = a + b
    qap = a + 1.0
    qam = a - 1.0
    c = 1.0
    d = 1.0 - qab * x / qap
    if abs(d) < 1e-30:
        d = 1e-30
    d = 1.0 / d
    result = d
    for m in range(1, max_iter + 1):
        m2 = 2 * m
        numerator = m * (b - m) * x / ((qam + m2) * (a + m2))
        d = 1.0 + numerator * d
        if abs(d) < 1e-30:
            d = 1e-30
        c = 1.0 + numerator / c
        if abs(c) < 1e-30:
            c = 1e-30
        d = 1.0 / d
        result *= d * c
        numerator = -(a + m) * (qab + m) * x / ((a + m2) * (qap + m2))
        d = 1.0 + numerator * d
        if abs(d) < 1e-30:
            d = 1e-30
        c = 1.0 + numerator / c
        if abs(c) < 1e-30:
            c = 1e-30
        d = 1.0 / d
        delta = d * c
        result *= delta
        if abs(delta - 1.0) < epsilon:
            return result
    raise ValueError("incomplete beta continued fraction did not converge")
