#pragma once

#include <cmath>

// Helper to get sign of a number
inline double get_sign(const double r)
{
    if (r != 0.0)
    {
        return std::copysign(1.0, r);
    }
    return 0.0;
}

// L1
inline double get_l1_rho(const double r)
{
    return std::fabs(r);
}

inline double get_l1_upsilon(const double r)
{
    return get_sign(r);
}

inline double get_l1_w(const double r)
{
    return 1.0 / std::fabs(r);
}

// L2
inline double get_l2_rho(const double r)
{
    return 0.5 * r * r;
}

inline double get_l2_upsilon(const double r)
{
    return r;
}

inline double get_l2_w(const double r)
{
    return 1.0;
}

// L1-L2
inline double get_l1l2_rho(const double r)
{
    const double rr = r * r;

    return 2.0 * (std::sqrt(1.0 + 0.5 * rr) - 1.0);
}

inline double get_l1l2_upsilon(const double r)
{
    const double rr = r * r;

    return r / std::sqrt(1.0 + 0.5 * rr);
}

inline double get_l1l2_w(const double r)
{
    const double rr = r * r;

    return 1.0 / std::sqrt(1.0 + 0.5 * rr);
}

// Lp
inline double get_lp_rho(const double r, const double nu)
{
    const double ar = std::fabs(r);

    return std::pow(ar, nu) / nu;
}

inline double get_lp_upsilon(const double r, const double nu)
{
    const double ar = std::fabs(r);

    if (r != 0.0)
    {
        return get_sign(r) * std::pow(ar, nu - 1.0);
    }

    return 0.0;
}

inline double get_lp_w(const double r, const double nu)
{
    return std::pow(std::fabs(r), nu - 2.0);
}

// Fair
inline double get_fair_rho(const double r, const double c)
{
    const double ar_c = std::fabs(r) / c;
    return c * c * (ar_c - std::log1p(ar_c));
}

inline double get_fair_upsilon(const double r, const double c)
{
    return r / (1.0 + std::fabs(r) / c);
}

inline double get_fair_w(const double r, const double c)
{
    return 1.0 / (1.0 + std::fabs(r) / c);
}

// Huber
inline double get_huber_rho(const double r, const double k)
{
    const double ar = std::fabs(r);

    if (ar < k)
    {
        return 0.5 * r * r;
    }

    return k * (ar - 0.5 * k);
}

inline double get_huber_upsilon(const double r, const double k)
{
    if (std::fabs(r) < k)
    {
        return r;
    }

    return k * get_sign(r);
}

inline double get_huber_w(const double r, const double k)
{
    if (std::fabs(r) < k)
    {
        return 1.0;
    }

    return k / std::fabs(r);
}

// Cauchy
inline double get_cauchy_rho(const double r, const double c)
{
    const double cc = c * c;
    const double rc = r / c;
    const double rcrc = rc * rc;

    return 0.5 * cc * std::log1p(rcrc);
}

inline double get_cauchy_upsilon(const double r, const double c)
{
    const double rc = r / c;
    const double rcrc = rc * rc;

    return r / (1.0 + rcrc);
}

inline double get_cauchy_w(const double r, const double c)
{
    const double rc = r / c;
    const double rcrc = rc * rc;

    return 1.0 / (1.0 + rcrc);
}

// Geman-McClure
inline double get_geman_mcclure_rho(const double r)
{
    const double rr = r * r;

    return 0.5 * rr / (1.0 + rr);
}

inline double get_geman_mcclure_upsilon(const double r)
{
    const double rr = r * r;

    return r / ((1.0 + rr) * (1.0 + rr));
}

inline double get_geman_mcclure_w(const double r)
{
    const double rr = r * r;

    return 1.0 / ((1.0 + rr) * (1.0 + rr));
}

// Welsch
inline double get_welsch_rho(const double r, const double c)
{
    const double rc = r / c;
    const double cc = c * c;
    const double rcrc = rc * rc;

    return 0.5 * cc * (1.0 - std::exp(-rcrc));
}

inline double get_welsch_upsilon(const double r, const double c)
{
    const double rc = r / c;
    const double rc2 = (rc) * (rc);

    return r * std::exp(-rc2);
}

inline double get_welsch_w(const double r, const double c)
{
    const double rc = r / c;
    const double rc2 = (rc) * (rc);

    return std::exp(-rc2);
}

// Tukey
inline double get_tukey_rho(const double r, const double c)
{
    const double ar = std::fabs(r);
    const double cc = c * c;

    if (ar >= c)
    {
        return cc / 6.0;
    }

    const double rc = r / c;
    const double rc2 = (rc) * (rc);
    const double temp = 1.0 - rc2;
    const double temp3 = temp * temp * temp;

    return cc / 6.0 * (1.0 - temp3);
}

inline double get_tukey_upsilon(const double r, const double c)
{
    const double ar = std::fabs(r);

    if (ar >= c)
    {
        return 0.0;
    }

    const double rc = r / c;
    const double rc2 = (rc) * (rc);
    const double temp = 1.0 - rc2;
    const double temp2 = temp * temp;

    return r * temp2;
}

inline double get_tukey_w(const double r, const double c)
{
    const double ar = std::fabs(r);

    if (ar >= c)
    {
        return 0.0;
    }

    const double rc = r / c;
    const double rc2 = (rc) * (rc);
    const double temp = 1.0 - rc2;
    const double temp2 = temp * temp;

    return temp2;
}

// Barron
inline double get_barron_rho(const double r, const double alpha, const double c)
{
    const double r_over_c = r / c;
    const double r_over_c_sq = r_over_c * r_over_c;
    const double abs_alpha_minus2 = std::fabs(alpha - 2.0);

    if (alpha <= -1e12)
    {
        return 1.0 - std::exp(-0.5 * r_over_c_sq);
    }

    if (alpha == 0.0)
    {
        return std::log(0.5 * r_over_c_sq + 1.0);
    }

    if (alpha == 2.0)
    {
        return 0.5 * r_over_c_sq;
    }

    return (abs_alpha_minus2 / alpha) * (std::pow((r_over_c_sq / abs_alpha_minus2) + 1.0, alpha / 2.0) - 1.0);
}

inline double get_barron_upsilon(const double r, const double alpha, const double c)
{
    const double r_over_c = r / c;
    const double r_over_c_sq = r_over_c * r_over_c;
    const double c_sq = c * c;
    const double abs_alpha_minus2 = std::fabs(alpha - 2.0);

    if (alpha <= -1e12)
    {
        return r / c_sq * std::exp(-0.5 * r_over_c_sq);
    }

    if (alpha == 0.0)
    {
        return 2.0 * r / (r * r + 2.0 * c_sq);
    }

    if (alpha == 2.0)
    {
        return r / c_sq;
    }

    return r / c_sq * std::pow((r_over_c_sq / abs_alpha_minus2) + 1.0, alpha / 2.0 - 1.0);
}

inline double get_barron_w(const double r, const double alpha, const double c)
{
    if (r == 0.0)
    {
        return 1.0;
    }

    return get_barron_upsilon(r, alpha, c) / r;
}

// Approximate partition function
inline double get_approximate_partition_function(
    const double tau_begin, const double tau_end, const double alpha, const double c, const double num_steps)
{
    const double step = (tau_end - tau_begin) / num_steps;
    double area = 0.0;

    for (int i = 0; i < static_cast<int>(num_steps); i++)
    {
        const double tau = tau_begin + (i + 0.5) * step;
        const double rho = get_barron_rho(tau, alpha, c);
        area += std::exp(-rho) * step;
    }

    return area;
}

// Truncated robust kernel
inline double get_truncated_robust_kernel(const double r, const double alpha, const double c, const double Z_tilde)
{
    return get_barron_rho(r, alpha, c) + std::log(c * Z_tilde);
}

// Modified probability distribution
inline double get_modified_probability_distribution(
    const double r, const double alpha, const double c, const double tau_begin, const double tau_end, const double num_steps)
{
    const double Z_tilde = get_approximate_partition_function(tau_begin, tau_end, alpha, 1.0, num_steps);
    return 1.0 / (c * Z_tilde) * std::exp(-get_barron_rho(r, alpha, c));
}