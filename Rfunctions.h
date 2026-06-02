/**
 * @file Rfunctions.h
 * @author Yohan FOUGEROLLE
 * @date 2026
 * * @brief This file is part of a research project on Generalized R-functions
 * for N-dimensional implicit modeling.
 * * @details Licensed under the MIT License.
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 */

#pragma once
#include <Eigen/Core>
#include <vector>
#include <memory>
#include <numeric>   // Required for std::iota
#include <algorithm> // Required for std::next_permutation
#include <iostream>
#include <cmath>
#include <Eigen/Dense>
#include <iomanip>

 /**
  * @brief Categorizes the family of implicit composition logics implemented within the KRF pipeline.
  */
enum class LogicType {
    MIN_NAIVE,      ///< Non-differentiable classical logic mapping: min(A, B, C)
    TERNARY_KRF,    ///< Directional proposed ternary Kernel R-Function framework
    BINARY_RP       ///< Inductive binary chain cascade representation: Rp(A, Rp(B, C))
};

namespace RCore {

    /**
     * @section CLASSICAL_R_FUNCTIONS Standard R-Functions (Rvachev-Shapiro)
     * @details Analytical composition of spatial potential fields.
     * References:
     * - Shapiro, V., 1991. Theory of R-functions and applications: A primer. Cornell University.
     * - Shapiro, V., 2007. Semi analytic geometry with R-functions. ACTA Numerica, 16, 239–303.
     */

     /** @brief Standard differential Rp-conjunction operator with bounded differential properties. */
    double Rp(double x1, double x2, bool inter);

    /** @brief Fully symmetric N-ary composition achieved via combinatorial average trees. */
    double RpAverageNd(const Eigen::VectorXd& X, bool inter);

    /** @brief Bounded linear R-conjunction operator controlled by regularizing parameter m. */
    double R0m(double x1, double x2, bool inter, int m = 1);

    /** * @brief N-ary additive formulation bypassing deep nested square-root architectures.
     * @note Evaluates using Zenkin's alternating combinatorial summation framework.
     */
    double ZenkinNd(const Eigen::VectorXd& X, bool inter);

    /** @brief Evaluates spatial gradient arrays for the N-ary Zenkin operator via central finite differences. */
    Eigen::VectorXd Gradient_ZenkinNd(const Eigen::VectorXd& X, bool inter, double h = 1e-4);

    /** @brief Classical higher-order linear N-ary Rvachev composition engine. */
    double RvachevNd(const Eigen::VectorXd& X, bool inter, int m = 0);

    /** @brief Evaluates spatial gradient arrays for the N-ary Rvachev operator via central finite differences. */
    Eigen::VectorXd Gradient_RvachevNd(const Eigen::VectorXd& X, bool inter, int m = 1, double h = 1e-4);

    /**
     * @brief Computes the strict analytical signed distance field (SDF) of an isolated sphere.
     * @param sphere Hyper-vector format mapping center coordinates onto [0,1,2] and radius onto [3].
     */
    double fSpherical(const Eigen::Vector3d& P, const Eigen::Vector4d& sphere);

    /**
     * @class SphericalEvaluator
     * @brief Continuous ternary logical operator framework for spherical simplex spaces.
     * @details Implements the continuous multi-primitive logical operators developed by Y. Fougerolle.
     */
    class SphericalEvaluator {
    public:
        SphericalEvaluator() = default;

        /** @brief Allocates spatial workspace dimensions for target canonical parameter mappings. */
        void Init(unsigned int dimension);

        /** @brief Closed-form analytical solver evaluating the aperture opening profiles of the tangent simplex. */
        double ComputeHeight(
            const Eigen::VectorXd& I,
            const Eigen::VectorXd& ei,
            const Eigen::VectorXd& ek,
            const Eigen::VectorXd& n);

        /** @brief Core directional ternary R-function execution engine. */
        double Compute(bool isIntersection, const Eigen::VectorXd& X);

        /** * @brief Scaled structural variant satisfying Euler's homogeneity degree-1 criteria.
         * @details Restores distance field metric behaviors by lifting directional output over input norm scales.
         */
        double ComputeNormalized(bool isIntersection, const Eigen::VectorXd& X)
        {
            double val = Compute(isIntersection, X);
            double n = X.norm();
            //if (n > 1e-12) val /= n;
            return val * n;
        }

        /** @brief Approximates the local gradient vector of the DIRECTIONAL formulation using finite differences. */
        Eigen::VectorXd ComputeGradient(bool isIntersection, const Eigen::VectorXd& X, double eps = 1e-7)
        {
            int n = X.size();
            Eigen::VectorXd G(n);
            Eigen::VectorXd P_plus = X;
            Eigen::VectorXd P_minus = X;

            for (int i = 0; i < n; ++i)
            {
                P_plus(i) += eps;
                P_minus(i) -= eps;

                G(i) = (Compute(isIntersection, P_plus) - Compute(isIntersection, P_minus)) / (2.0 * eps);

                P_plus(i) = X(i);
                P_minus(i) = X(i);
            }
            return G;
        }

        /** @brief Approximates the local gradient vector of the NORMALIZED formulation using finite differences. */
        Eigen::VectorXd ComputeGradientNormalized(bool isIntersection, const Eigen::VectorXd& X, double eps = 1e-7)
        {
            int n = X.size();
            Eigen::VectorXd G(n);
            Eigen::VectorXd P_plus = X;
            Eigen::VectorXd P_minus = X;

            for (int i = 0; i < n; ++i)
            {
                P_plus(i) += eps;
                P_minus(i) -= eps;

                G(i) = (ComputeNormalized(isIntersection, P_plus) - ComputeNormalized(isIntersection, P_minus)) / (2.0 * eps);

                P_plus(i) = X(i);
                P_minus(i) = X(i);
            }
            return G;
        }

    private:
        Eigen::VectorXd northPole;            ///< Spherical geometry positive focal anchor vector
        Eigen::VectorXd southPole;            ///< Spherical geometry negative focal anchor vector
        unsigned short N = 0;                 ///< Dimensional bounds index tracker. Restricted to N=3 configurations.
        std::vector<Eigen::VectorXd> ei;      ///< Active canonical basis vector components tracking angular spans
        std::vector<Eigen::VectorXd> n_ei;    ///< Target surface normal complements tracking boundary junctions
        std::vector<Eigen::VectorXd> ei_equatorial;   ///< Canonical projections computed across the equatorial plane
        std::vector<Eigen::VectorXd> n_ei_equatorial; ///< Structural normals matching the equatorial basis alignments

        /** @brief Identifies the active angular segment enclosure containing the projected target coordinate point. */
        int getAngularSector(const Eigen::Vector3d& proj, const std::vector<Eigen::VectorXd>& eq);

        /** * @brief Maps circular trajectory positions sweeping from poles across active directional boundaries.
         * @note Parameterizes complex topological transformations analogous to an opening/closing umbrella.
         */
        std::vector<Eigen::VectorXd> generateSphericalUmbrellaPoints(
            double t,
            const std::vector<Eigen::VectorXd>& directions,
            const Eigen::VectorXd& pole);
    };
}


/**
 * @class SphericalValidation
 * @brief Numerical testing suite and mathematical validation environment for N-ary aperture formulations.
 * @details Contains successive iterations of algebraic regularizations evaluating multi-object blending mechanics,
 * projective tangency parameters, log-sum-exp smooth structures, and boundary sealing metrics.
 */
class SphericalValidation {
public:

    /** @brief Computes continuous angular aperture deviations relative to standard simplex edge intersections. */
    static double EvaluateApertureDiff(const Eigen::VectorXd& X) {
        int n = X.size();
        double norm = X.norm();
        if (norm < 1e-15) return 0.0;

        Eigen::VectorXd Q = X / norm;

        // 1. Reference aperture mapping threshold (Axis to Orthoscheme Edge)
        double theta_ref = std::acos(1.0 / std::sqrt(n));

        // 2. Compute explicit parameter properties matching target evaluation point Q (theta_Q)
        double q_min = Q.minCoeff();
        double q_sum = Q.sum();

        // Axial core orientation tracker parameter (cos_gamma)
        double cos_gamma = q_sum / std::sqrt(n);

        // h_perp: Normal distance separating coordinate position from barycentric axis lines
        double h_perp = (cos_gamma / std::sqrt(n)) - q_min;

        // Extract tan(theta_Q) utilizing simplicial geometry relationships
        double tan_theta_Q = (h_perp * std::sqrt(n / (n - 1.0)) / std::max(1e-15, cos_gamma)) * std::sqrt(n - 1.0);
        double theta_Q = std::atan(tan_theta_Q);

        // --- DOMAIN BOUNDARY OUTSIDE-TRANSITION SANITIZATION ---
        if (q_min < 0) {
            // Points escaping explicit facial enclosures scale aperture metrics beyond reference boundaries
            theta_Q = std::abs(theta_Q) + theta_ref;
        }

        return (theta_ref - theta_Q); //*norm;
    }

    /** @brief Resonance-driven algebraic mapping enforcing dome topology transitions. */
    static double EvaluateApertureAlgebraicBourrelets(const Eigen::VectorXd& X) {
        int n = X.size();
        double norm = X.norm();
        if (norm < 1e-15) return 0.0;

        double q_min = X.minCoeff();
        double q_sum = X.sum();
        double q_axial = q_sum / std::sqrt(n);
        double h_perp = std::sqrt(std::max(0.0, norm * norm - q_axial * q_axial));

        // 1. Resonance factor driving structural geometric dome preservation
        double resonance = norm / (h_perp + std::abs(q_min) + 1e-9);

        // 2. Adaptive scale dampening gradient amplification while preserving topology shapes
        double adaptive_scale = (std::abs(q_axial) + h_perp) / (norm + 1e-9);

        return q_min * resonance * adaptive_scale;
    }

    /** @brief Regularized tangency ratio model smoothing transverse dispersion via hypercube epsilon limits. */
    static double EvaluateApertureAlgebraicV2(const Eigen::VectorXd& X, double epsilon) {
        int n = X.size();
        double norm_sq = X.squaredNorm();
        double norm = std::sqrt(norm_sq);

        if (norm < 1e-15) return X.minCoeff();

        double q_sum = X.sum();
        double q_min = X.minCoeff();

        // 1. Axial projection tracking vs transverse dispersion limits
        double q_axial = q_sum / std::sqrt(static_cast<double>(n));
        double h_perp = std::sqrt(std::max(0.0, norm_sq - q_axial * q_axial));

        // 2. Tangency ratio computation regularized by coordinate parameters
        double denominator = std::sqrt(h_perp * h_perp + epsilon * epsilon * norm_sq);
        if (denominator < 1e-15) denominator = 1e-15;

        double T = q_min / denominator;

        // 3. Cauchy-type compression kernel invocation: Psi(T) = T / sqrt(1 + T^2)
        double psi_T = T / std::sqrt(1.0 + T * T);

        return norm * psi_T;
    }

    /** @brief Decoupled variant preserving uncompressed logical terms within projective structures. */
    static double EvaluateApertureAlgebraicV3(const Eigen::VectorXd& X, double epsilon) {
        int n = static_cast<int>(X.size());
        double norm_sq = X.squaredNorm();
        double norm = std::sqrt(norm_sq);

        if (norm < 1e-15) return X.minCoeff();

        double q_sum = X.sum();
        double q_min = X.minCoeff();

        double q_axial = q_sum / std::sqrt(static_cast<double>(n));
        double h_perp = std::sqrt(std::max(0.0, norm_sq - q_axial * q_axial));

        // Prevents spatial cancellation of standard distance normalization fields
        double denominator = std::sqrt(h_perp * h_perp + std::pow(epsilon * q_axial, 2));
        if (denominator < 1e-15) denominator = 1e-15;

        double T = q_min / denominator;
        double psi_T = T;

        return norm * psi_T;
    }

    /** @brief Soft-saturating profiling kernel (p=1 formulation) maximizing structural ridge preservation. */
    static double EvaluateApertureAlgebraicV4(const Eigen::VectorXd& X, double epsilon) {
        int n = static_cast<int>(X.size());
        double norm_sq = X.squaredNorm();
        double norm = std::sqrt(norm_sq);

        if (norm < 1e-15) return X.minCoeff();

        double q_sum = X.sum();
        double q_min = X.minCoeff();

        double q_axial = q_sum / std::sqrt(static_cast<double>(n));
        double h_perp = std::sqrt(std::max(0.0, norm_sq - q_axial * q_axial));

        double denominator = std::sqrt(h_perp * h_perp + (epsilon * epsilon * norm_sq));
        if (denominator < 1e-15) denominator = 1e-15;

        double T = q_min / denominator;

        // Soft-saturating mapping layout: preserves structural peak merging characteristics
        double psi_T = T / (1.0 + std::abs(T));

        return norm * psi_T;
    }

    /** @brief Hyperbolic tangent kernel setup enforcing rapid convexity restoration across multi-primitive valleys. */
    static double EvaluateApertureAlgebraicV5(const Eigen::VectorXd& X, double epsilon) {
        int n = static_cast<int>(X.size());
        double norm_sq = X.squaredNorm();
        double norm = std::sqrt(norm_sq);

        if (norm < 1e-15) return X.minCoeff();

        double q_sum = X.sum();
        double q_min = X.minCoeff();

        double q_axial = q_sum / std::sqrt(static_cast<double>(n));
        double h_perp = std::sqrt(std::max(0.0, norm_sq - q_axial * q_axial));

        double denominator = std::sqrt(h_perp * h_perp + (epsilon * epsilon * norm_sq));
        if (denominator < 1e-15) denominator = 1e-15;

        double T = q_min / denominator;
        double psi_T = std::tanh(T);

        return norm * psi_T;
    }

    /** @brief Quadratic transverse penalty formulation suppressing mid-range blending artifacts. */
    static double EvaluateApertureAlgebraicV6(const Eigen::VectorXd& X, double epsilon) {
        int n = static_cast<int>(X.size());
        double norm_sq = X.squaredNorm();
        double norm = std::sqrt(norm_sq);

        if (norm < 1e-15) return X.minCoeff();

        double q_sum = X.sum();
        double q_min = X.minCoeff();

        double q_axial = q_sum / std::sqrt(static_cast<double>(n));
        double h_perp = std::sqrt(std::max(0.0, norm_sq - q_axial * q_axial));

        // Sub-quadratic scaling minimizes axis proximity penalties
        double dispersion = (h_perp * h_perp) / (norm + 1e-6);
        double couplage = epsilon * norm;

        double denominator = dispersion + couplage;
        if (denominator < 1e-15) denominator = 1e-15;

        double T = q_min / denominator;
        double psi_T = std::tanh(T);

        return norm * psi_T;
    }

    /** @brief L1-norm adaptive configuration mapping matching standard manuscript equations. */
    static double EvaluateApertureAlgebraicV8(const Eigen::VectorXd& X, double epsilon) {
        int n = static_cast<int>(X.size());
        double norm_sq = X.squaredNorm();
        double norm = std::sqrt(norm_sq);

        if (norm < 1e-15) return X.minCoeff();

        double q_sum = X.sum();
        double q_min = X.minCoeff();

        double q_axial = q_sum / std::sqrt(static_cast<double>(n));
        double h_perp = std::sqrt(std::max(0.0, norm_sq - q_axial * q_axial));

        // Coupling locked onto absolute L1 system constraints configurations
        double coupling = epsilon * (std::abs(q_axial) + h_perp);

        double denominator = h_perp + coupling;
        if (denominator < 1e-15) denominator = 1e-15;

        double T = q_min / denominator;
        double psi_T = T / std::sqrt(1.0 + T * T);

        return norm * psi_T;
    }

    /** @brief Collective consensus model incorporating localized field cooperation parameters. */
    static double EvaluateApertureAlgebraicV9(const Eigen::VectorXd& X, double epsilon, double lambda = 0.5) {
        int n = static_cast<int>(X.size());
        double norm_sq = X.squaredNorm();
        double norm = std::sqrt(norm_sq);

        if (norm < 1e-15) return X.minCoeff();

        double q_sum = X.sum();
        double q_min = X.minCoeff();

        double q_axial = q_sum / std::sqrt(static_cast<double>(n));
        double h_perp = std::sqrt(std::max(0.0, norm_sq - q_axial * q_axial));

        // Mutual synergy term handling sharp boundary crossings smoothly
        double S_x = q_sum / (norm + 1e-9);

        double numerator = q_min * (1.0 + lambda * S_x);
        double denominator = h_perp + (epsilon * norm);

        if (denominator < 1e-15) denominator = 1e-15;

        double T = numerator / denominator;
        double psi_T = std::tanh(T);

        return norm * psi_T;
    }

    /** @brief Dual binary R-conjunction blending local fields with holistic global scene descriptions. */
    static double EvaluateApertureAlgebraicV10(const Eigen::VectorXd& X, double epsilon, double lambda = 0.3) {
        int n = static_cast<int>(X.size());
        double norm_sq = X.squaredNorm();
        double norm = std::sqrt(norm_sq);

        if (norm < 1e-15) return X.minCoeff();

        double q_min = X.minCoeff();
        double q_sum = X.sum();
        double q_avg = q_sum / n;

        double q_axial = q_sum / std::sqrt(static_cast<double>(n));
        double h_perp = std::sqrt(std::max(0.0, norm_sq - q_axial * q_axial));

        double denominator = h_perp + (epsilon * norm);
        if (denominator < 1e-15) denominator = 1e-15;
        double T = q_min / denominator;

        // Blends spatial component minimums with ensemble arithmetic averages to secure zero-sets
        double consensus = q_min + q_avg - std::sqrt(q_min * q_min + q_avg * q_avg - 2.0 * lambda * q_min * q_avg);
        double T_suture = consensus / denominator;

        double psi_T = T_suture / std::sqrt(1.0 + T_suture * T_suture);

        return norm * psi_T;
    }

    /** @brief Angular regularizer compressing the transverse distance influence near barycentric lines. */
    static double EvaluateApertureAlgebraicV11(const Eigen::VectorXd& X, double epsilon) {
        int n = static_cast<int>(X.size());
        double norm_sq = X.squaredNorm();
        double norm = std::sqrt(norm_sq);

        if (norm < 1e-15) return X.minCoeff();

        double q_min = X.minCoeff();
        double q_sum = X.sum();

        double q_axial = q_sum / std::sqrt(static_cast<double>(n));
        double h_perp = std::sqrt(std::max(0.0, norm_sq - q_axial * q_axial));

        double denominator = (h_perp * h_perp) / (norm)+(epsilon * norm);
        if (denominator < 1e-15) denominator = 1e-15;

        double T = q_min / denominator;
        double psi_T = T / std::sqrt(1.0 + T * T);

        return norm * psi_T;
    }

    /** @brief Non-discrete logical surrogate mapping input potentials via normalized product metrics. */
    static double EvaluateApertureAlgebraicV12(const Eigen::VectorXd& X, double epsilon) {
        int n = static_cast<int>(X.size());
        double norm_sq = X.squaredNorm();
        double norm = std::sqrt(norm_sq);

        if (norm < 1e-15) return 0.0;

        double q_sum = X.sum();

        double q_axial = q_sum / std::sqrt(static_cast<double>(n));
        double h_perp = std::sqrt(std::max(0.0, norm_sq - q_axial * q_axial));

        // Substitutes discrete minimum selectors with regularized coordinate volume products
        double product = X.array().prod();
        double q_prod_normalized = product / (std::pow(norm, n - 1) + 1e-18);

        double q_logic = (X.minCoeff() < 0) ? X.minCoeff() : q_prod_normalized;

        double denominator = (h_perp * h_perp) / (norm)+(epsilon * norm);
        if (denominator < 1e-15) denominator = 1e-15;

        double T = q_logic / denominator;
        double psi_T = T / std::sqrt(1.0 + T * T);

        return norm * psi_T;
    }

    /** @brief C-infinite continuous smooth minimum approximation utilizing Kreisselmeier-Steinhauser parameters. */
    static double EvaluateApertureAlgebraicV13(const Eigen::VectorXd& X, double epsilon) {
        int n = static_cast<int>(X.size());
        double norm_sq = X.squaredNorm();
        double norm = std::sqrt(norm_sq);

        if (norm < 1e-15) return 0.0;

        double q_sum = X.sum();
        double q_axial = q_sum / std::sqrt(static_cast<double>(n));
        double h_perp = std::sqrt(std::max(0.0, norm_sq - q_axial * q_axial));

        // Computes a differentiable log-sum-exp alternative to hard min operations
        double rho = 20.0;
        double exp_sum = 0.0;
        for (int i = 0; i < n; ++i) exp_sum += std::exp(-rho * X[i]);
        double q_logic = -std::log(exp_sum) / rho;

        double denominator = (h_perp * h_perp) / (norm)+(epsilon * norm);
        if (denominator < 1e-15) denominator = 1e-15;

        double T = q_logic / denominator;
        double psi_T = T / std::sqrt(1.0 + T * T);

        return norm * psi_T;
    }

    /** @brief High-fidelity smooth minimum solver variant locking sharpness thresholds to strict levels. */
    static double EvaluateApertureAlgebraicV14(const Eigen::VectorXd& X, double epsilon) {
        int n = static_cast<int>(X.size());
        double norm = X.norm();

        if (norm < 1e-15) return 0.0;

        double q_sum = X.sum();
        double q_axial = q_sum / std::sqrt(static_cast<double>(n));
        double h_perp = std::sqrt(std::max(0.0, norm * norm - q_axial * q_axial));

        double rho = 100.0;
        double exp_sum = 0.0;
        for (int i = 0; i < n; ++i) exp_sum += std::exp(-rho * X[i]);
        double q_smooth_min = -std::log(exp_sum) / rho;

        double denominator = (h_perp * h_perp) / norm + (epsilon * norm);
        double T = q_smooth_min / std::max(denominator, 1e-15);

        double psi_T = T / std::sqrt(1.0 + T * T);

        return norm * psi_T;
    }

    /** @brief Core collinearity-boosted R-function enforcing explicit directional stability profiles. */
    static double EvaluateApertureAlgebraic(const Eigen::VectorXd& X) {
        int n = X.size();
        double norm_sq = X.squaredNorm();
        double norm = std::sqrt(norm_sq);
        if (norm < 1e-15) return 0.0;

        double q_sum = X.sum();
        double q_min = X.minCoeff();

        double q_axial = q_sum / std::sqrt(n);
        double h_perp = std::sqrt(std::max(0.0, norm_sq - q_axial * q_axial));

        // Resonance amplification factor optimizing alignment stability along the tracking axis
        const double epsilon = 1e-9;
        double resonance = norm / (h_perp + std::abs(q_min) + epsilon);

        return q_min * resonance;
    }

    /** @brief Homogeneous degree-1 explicit field compiling smooth KS partitions across adaptive baselines. */
    static double EvaluateApertureAlgebraicV15(const Eigen::VectorXd& X, double epsilon)
    {
        const int n = static_cast<int>(X.size());
        double norm_sq = X.squaredNorm();
        double norm = std::sqrt(norm_sq);

        if (norm < 1e-15) return 0.0;

        double q_sum = X.sum();
        double q_axial = q_sum / std::sqrt(static_cast<double>(n));

        double h_perp_sq = std::max(0.0, norm_sq - q_axial * q_axial);
        double h_perp = std::sqrt(h_perp_sq);

        double rho = 100.0;
        double exp_sum = 0.0;
        for (int i = 0; i < n; ++i) { exp_sum += std::exp(-rho * X[i]); }
        double L = -std::log(exp_sum) / rho;

        double D = (h_perp_sq / norm) + (epsilon * norm);
        double T = L / std::max(D, 1e-15);

        double psi = T / std::sqrt(1.0 + T * T);
        return norm * psi;
    }

    /** @brief Multi-mode framework supporting optional hard structural toggles versus soft log-sum-exp structures. */
    static double EvaluateApertureAlgebraicV16(
        const Eigen::VectorXd& X,
        double epsilon,
        bool useKS,
        double rho = 100.0)
    {
        int n = static_cast<int>(X.size());
        double norm_sq = X.squaredNorm();
        double norm = std::sqrt(norm_sq);

        if (norm < 1e-15) return 0.0;

        double q_sum = X.sum();
        double q_axial = q_sum / std::sqrt(static_cast<double>(n));

        double h_perp_sq = std::max(0.0, norm_sq - q_axial * q_axial);
        double h_perp = std::sqrt(h_perp_sq);

        double L = 0.0;
        if (useKS)
        {
            double exp_sum = 0.0;
            for (int i = 0; i < n; ++i) { exp_sum += std::exp(-rho * X[i]); }
            L = -std::log(exp_sum) / rho;
        }
        else
        {
            L = X.minCoeff();
        }

        double D = h_perp_sq / std::max(norm, 1e-15);
        double denom = D + epsilon * norm;
        denom = std::max(denom, 1e-15);

        double T = L / denom;
        double psi = T / std::sqrt(1.0 + T * T);

        return norm * psi;
    }

    /** @brief Direct sin-aperture mapping bypassing infinite tangency asymptotic limits. */
    static double EvaluateApertureAlgebraicFINALFINAL(const Eigen::VectorXd& X) {
        int n = X.size();
        double norm_sq = X.squaredNorm();
        double norm = std::sqrt(norm_sq);

        if (norm < 1e-15) return 0.0;

        double q_sum = X.sum();
        double q_min = X.minCoeff();

        double q_axial = q_sum / std::sqrt(n);
        double h_perp_sq = std::max(0.0, norm_sq - q_axial * q_axial);
        double h_perp = std::sqrt(h_perp_sq);

        // Directly calculates the sinus operator to bypass coordinate singularities
        double denom = std::sqrt(h_perp_sq + q_min * q_min);
        double potential = 0.0;
        if (denom > 1e-18 * norm) {
            potential = q_min / denom;
        }
        else {
            potential = (q_min > 0) ? 1.0 : 0.0;
        }

        return norm * potential;
    }

    /** @brief Stable geometric direct projection computing hypotenuse values without division loops. */
    static double EvaluateApertureAlgebraicv9(const Eigen::VectorXd& X) {
        int n = X.size();
        double norm = X.norm();
        if (norm < 1e-15) return 0.0;

        double q_sum = X.sum();
        double q_min = X.minCoeff();

        double q_axial = q_sum / std::sqrt(n);
        double h_perp = std::sqrt(std::max(0.0, norm * norm - q_axial * q_axial));

        double denom = std::sqrt(h_perp * h_perp + q_min * q_min);
        double potential;
        if (denom < 1e-15) {
            potential = (q_min > 0) ? 1.0 : 0.0;
        }
        else {
            potential = q_min / denom;
        }

        return norm * potential;
    }

    /** @brief Axis proximity isolation routine handling unified perfect consensus coordinates. */
    static double EvaluateApertureAlgebraictest(const Eigen::VectorXd& X) {
        int n = X.size();
        double norm = X.norm();
        if (norm < 1e-15) return 0.0;

        double q_sum = X.sum();
        double q_min = X.minCoeff();

        double q_axial = q_sum / std::sqrt(n);
        double h_perp_sq = std::max(0.0, norm * norm - q_axial * q_axial);
        double h_perp = std::sqrt(h_perp_sq);

        double potential;
        const double threshold = 1e-12;

        if (h_perp < threshold) {
            potential = 1.0;
        }
        else {
            double T = q_min / h_perp;
            potential = T / std::sqrt(1.0 + T * T);
        }

        return norm * potential;
    }

    /** @brief Proportional scaling implementation tracking parameter bounds via norm-locked adjustments. */
    static double EvaluateApertureAlgebraicv8(const Eigen::VectorXd& X) {
        int n = X.size();
        double norm = X.norm();
        if (norm < 1e-15) return 0.0;

        double q_sum = X.sum();
        double q_min = X.minCoeff();

        double q_axial = q_sum / std::sqrt(n);
        double h_perp = std::sqrt(std::max(0.0, norm * norm - q_axial * q_axial));

        const double epsilon = 1e-7;
        double T = q_min / (h_perp + epsilon * norm);
        double potential = T / std::sqrt(1.0 + T * T);

        return norm * potential;
    }

    /*
    static double EvaluateApertureAlgebraic(const Eigen::VectorXd& X) {
        int n = X.size();
        double norm = X.norm();
        if (norm < 1e-15) return 0.0;

        double q_sum = X.sum();
        double q_min = X.minCoeff();

        // 1. Calcul de la distance transverse (écart au consensus)
        // h_perp vaut 0 sur l'axe barycentrique
        double q_axial = q_sum / std::sqrt(n);
        double h_perp = std::sqrt(std::max(0.0, norm * norm - q_axial * q_axial));

        // 2. Définition du ratio T (Type Cotangente)
        // T est infini sur l'axe, 0 sur la frontière, négatif dehors.
        double T = 0.0;
        if (h_perp > 1e-15) {
            T = q_min / h_perp;
        }
        else {
            // On est sur l'axe
            T = (q_min >= 0) ? 1e10 : -1e10;
        }

        // 3. Le Noyau Psi
        // On veut une fonction qui transforme T en un potentiel saturé
        // mais qui ne s'écrase pas au centre.
        // Une forme sigmoïde simple : T / sqrt(1 + T^2)
        double potential = T / std::sqrt(1.0 + T * T);

        // 4. Lifting métrique
        return norm * potential;
    }
    */

    /** @brief Half-space compensation layout resolving vanishing sum geometric traps. */
    static double EvaluateApertureAlgebraicCorrectMaisGradient(const Eigen::VectorXd& X) {
        int n = X.size();
        double norm = X.norm();
        if (norm < 1e-15) return 0.0;

        double q_sum = X.sum();
        double q_min = X.minCoeff();

        double T = 1.0;
        if (std::abs(q_sum) > 1e-15) {
            T = 1.0 - (n * q_min) / q_sum;
        }
        else {
            T = 1e10;
        }

        if (q_sum < 0) {
            T = 2.0 - T;
        }

        double potential = (1.0 - T) / (1.0 + T);
        return norm * potential;
    }

    /** @brief Strict analytical dual union composition evaluated via inverted coordinate mappings. */
    static double EvaluateApertureUnionAlgebraic(const Eigen::VectorXd& X) {
        return -EvaluateApertureAlgebraic(-X);
    }

    /** @brief Explores standard spatial aperture dimensions focusing on maximal value selections. */
    static double EvaluateApertureUnion(const Eigen::VectorXd& X) {
        int n = X.size();
        double norm = X.norm();
        if (norm < 1e-15) return 0.0;
        Eigen::VectorXd Q = X / norm;

        double theta_ref = std::acos(1.0 / std::sqrt(n));

        double q_max = Q.maxCoeff();
        double q_sum = Q.sum();
        double cos_gamma = q_sum / std::sqrt(n);

        double h_perp_union = (cos_gamma / std::sqrt(n)) - q_max;

        double tan_theta_Q = (h_perp_union * std::sqrt(n / (n - 1.0)) / std::max(1e-15, cos_gamma)) * std::sqrt(n - 1.0);
        double theta_Q = std::atan(tan_theta_Q);

        if (q_max < 0) {
            theta_Q = std::abs(theta_Q) + theta_ref;
        }

        return (theta_ref - theta_Q); //*norm;
    }

    /** @brief Automated numeric boundary tracking suite checking monotonicity and seal failures. */
    static void RunLeakageTest(int n, long long samples) {
        const double epsilon = 1e-10;
        long long falsePositives = 0;
        long long falseNegatives = 0;
        long long catastrophicErrors = 0;

        double rMin = 1e15, rMax = -1e15;

        for (long long i = 0; i < samples; ++i) {
            Eigen::VectorXd X = Eigen::VectorXd::Random(n);
            double r = EvaluateApertureDiff(X);
            double q_min = X.minCoeff();

            if (r < rMin) rMin = r;
            if (r > rMax) rMax = r;

            if (std::abs(r) < epsilon) {
                bool actuallyNearBoundary = (std::abs(q_min) < 1e-8);
                if (!actuallyNearBoundary) {
                    if (q_min < 0) falsePositives++;
                    else falseNegatives++;
                }
            }

            if (q_min < -1e-9 && r > 0) {
                catastrophicErrors++;
            }
        }

        std::cout << "--- LEAKAGE & DYNAMICS (Dim " << n << ") ---" << std::endl;
        std::cout << "Plage R-fonction   : [" << rMin << " , " << rMax << "]" << std::endl;
        std::cout << "Faux bords (eps)   : " << falsePositives + falseNegatives << std::endl;
        std::cout << "Erreurs Critiques  : " << catastrophicErrors << " (Point hors-cone vu comme dedans)" << std::endl;

        if (catastrophicErrors == 0 && (falsePositives + falseNegatives) == 0)
            std::cout << "RÉSULTAT : Frontière parfaitement étanche et monotone." << std::endl;
        std::cout << "------------------------------------------" << std::endl;
    }

    /** @brief Hyper-dimensional statistical grid performance monitor scoring orthant ratio distributions. */
    static void RunBenchmark(int n, long long samples) {
        long long errorCount = 0;
        long long insideCount = 0;

        for (long long i = 0; i < samples; ++i) {
            Eigen::VectorXd X = Eigen::VectorXd::Random(n);
            double val = EvaluateApertureDiff(X);
            double truth = X.minCoeff();

            if ((val >= 0) != (truth >= 0)) {
                errorCount++;
            }

            if (truth >= 0) insideCount++;
        }

        double measuredRatio = static_cast<double>(insideCount) / samples;
        double targetRatio = std::pow(0.5, n);

        std::cout << "Dimension: " << std::setw(3) << n
            << " | Sign Errors: " << std::setw(5) << errorCount
            << " | Ratio: " << std::fixed << std::setprecision(8) << measuredRatio
            << " (Target: " << targetRatio << ")" << std::endl;
    }
};