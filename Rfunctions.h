/**
 * @file Rfunctions.h
 * @author Yohan FOUGEROLLE
 * @date 2026
 * @brief This file is part of a research project on Generalized R-functions
 * for N-dimensional implicit modeling.
 * @details Licensed under the MIT License.
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
    MIN_NAIVE,      ///< Non-differentiable classical logic mapping: min(A, B, C) or max(A, B, C)
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

