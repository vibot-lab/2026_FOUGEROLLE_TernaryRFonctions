/**
 * @file Rfunctions.cpp
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


#include "Rfunctions.h"
#include <cmath>
#include <algorithm>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace std;

namespace RCore {

    // =========================================================================
    // ANALYTICAL IMPLICIT PRIMITIVES
    // =========================================================================

    double fSpherical(const Eigen::Vector3d& P, const Eigen::Vector4d& sphere) {
        // Computes the strict algebraic signed distance field (SDF) to a hyper-sphere surface
        return (P - sphere.head<3>()).norm() - sphere[3];
    }

    // =========================================================================
    // CLASSICAL BINARY R-FUNCTIONS (RVACHEV-SHAPIRO)
    // =========================================================================

    double Rp(double x1, double x2, bool inter) {
        // Standard non-differentiable Rvachev conjonction/disjonction utilizing planar hypotenuse tracking
        double h = std::hypot(x1, x2);
        return inter ? (x1 + x2 - h) : (x1 + x2 + h);
    }

    double R0m(double x1, double x2, bool inter, int m) {
        double h = std::hypot(x1, x2);
        double r_base = inter ? (x1 + x2 - h) : (x1 + x2 + h);

        if (m == 0) return r_base;

        // Cm regularization: multiplying by h^m dampens and smooths out 
        // the non-differentiable ridge lines (sutures) at local intersection corners
        return std::pow(h, m) * r_base;
    }

    // =========================================================================
    // N-ARY COMPOSITIONS VIA TOPOLOGICAL AVERAGING TREE PACKETS
    // =========================================================================

    double RpAverageNd(const Eigen::VectorXd& X, bool inter) {
        const int n = static_cast<int>(X.size());

        // Edge conditions: isolated systems require no geometric composition steps
        if (n == 0) return 0.0;
        if (n == 1) return X[0];

        // 1. Initialize structural index sequence arrays tracking lexical permutations
        std::vector<int> indices(n);
        std::iota(indices.begin(), indices.end(), 0); // Fills buffer with consecutive sequences {0, 1, ..., n-1}

        double total_potential_sum = 0.0;
        uint64_t permutation_count = 0;

        // 2. Systematic Permutation Traversal over the Symmetric Group S_n.
        // Restores perfect spatial isotropy and eliminates binary chain structural bias 
        // by evaluating every topological tree configuration permutation.
        do {
            // Evaluate a right-leaning recursive cascade from bottom leaves up to the root element
            double current_chain_val = X[indices[n - 1]];
            for (int k = n - 2; k >= 0; --k) {
                current_chain_val = RCore::Rp(X[indices[k]], current_chain_val, inter);
            }

            total_potential_sum += current_chain_val;
            permutation_count++;

        } while (std::next_permutation(indices.begin(), indices.end()));

        // 3. Normalize: compute the absolute isotropic geometric average field
        return total_potential_sum / static_cast<double>(permutation_count);
    }

    // =========================================================================
    // N-ARY CLOSED-FORM RADICAL SUMMATIONS (ZENKIN MODEL)
    // =========================================================================

    double ZenkinNd(const Eigen::VectorXd& X, bool inter) {
        const int n = static_cast<int>(X.size());
        if (n == 0) return 0.0;
        if (n == 1) return X[0];

        double total_sum = 0.0;
        const long long total_combinations = 1LL << n; // Computes 2^n power-set bounds configurations

        // Iterate across the combinatorial power-set space spanning 1 to 2^n - 1
        for (long long i = 1; i < total_combinations; ++i) {
            int k = 0;
            double squared_sum = 0.0;
            int last_j = 0;

            // Extract set membership indicators utilizing efficient bit-shift mask checks
            for (int j = 0; j < n; ++j) {
                if ((i >> j) & 1) {
                    squared_sum += X[j] * X[j];
                    last_j = j;
                    k++;
                }
            }

            double radical = std::sqrt(squared_sum);

            if (inter) {
                // INTERSECTION (N-ary Conjunction Formulation)
                // Maps alternating signs onto k-tuple Euclidean norms:
                // k = 1  => alpha_1 = sum(x_i)
                // k >= 2 => alpha_k = (-1)^(k+1) * sqrt(sum(x_i^2))
                if (k == 1) {
                    total_sum += X[last_j];
                }
                else {
                    double sign = (k % 2 == 1) ? 1.0 : -1.0;
                    total_sum += sign * radical;
                }
            }
            else {
                // UNION (N-ary Disjunction Pass)
                // Handled downstream via De Morgan's strict duality mapping logic
            }
        }

        // -----------------------------------------------------------------
        // DE MORGAN DUALITY REFLECTION PIPELINE
        // -----------------------------------------------------------------
        // Evaluates the continuous Union space via De Morgan's reflection: Z_cup(X) = -Z_cap(-X).
        // This is mathematically equivalent to the alpha_k -> (-1)^k * alpha_k mapping rule,
        // while remaining robust and conserving strict potential fields orientation signatures.
        if (!inter) {
            Eigen::VectorXd minusX = -X;
            return -ZenkinNd(minusX, true);
        }

        return total_sum;
    }

    Eigen::VectorXd Gradient_ZenkinNd(const Eigen::VectorXd& X, bool inter, double h) {
        const int dim = static_cast<int>(X.size());
        Eigen::VectorXd grad(dim);

        Eigen::VectorXd X_plus = X;
        Eigen::VectorXd X_minus = X;

        // Central finite differences routine achieving high-accuracy O(h^2) partial spatial derivatives
        for (int i = 0; i < dim; ++i) {
            X_plus[i] += h;
            X_minus[i] -= h;

            double f_plus = ZenkinNd(X_plus, inter);
            double f_minus = ZenkinNd(X_minus, inter);

            grad[i] = (f_plus - f_minus) / (2.0 * h);

            X_plus[i] = X[i];
            X_minus[i] = X[i];
        }

        return grad;
    }

    // =========================================================================
    // N-ARY POLYNOMIAL FIELD PRODUCT FORMULATIONS (RVACHEV GENERATION)
    // =========================================================================

    double RvachevNd(const Eigen::VectorXd& X, bool inter, int m) {
        const int n = X.size();
        if (n == 0) return 0.0;

        // Perform parallel array absolute value mapping blocks via Eigen array mechanics
        auto absX = X.array().abs();
        Eigen::VectorXd Diff1 = (X.array() - absX).matrix();
        Eigen::VectorXd Diff2 = (absX - X.array()).matrix();
        Eigen::VectorXd Sum = (X.array() + absX).matrix();

        double S = 0.0;
        double P = 1.0;
        double signM = (m % 2 == 0) ? 1.0 : -1.0;

        for (int i = 0; i < n; ++i) {
            // Low-level Pow optimization: substitute costly calls with local copies when m=1
            double xPowM = (m == 1) ? X[i] : std::pow(X[i], m);
            if (inter) {
                S += signM * xPowM * Diff1[i];
                P *= xPowM * Sum[i];
            }
            else {
                S += xPowM * Sum[i];
                P *= signM * xPowM * Diff2[i];
            }
        }
        return inter ? (S + P) : (S - P);
    }

    Eigen::VectorXd Gradient_RvachevNd(const Eigen::VectorXd& X, bool inter, int m, double h) {
        const int dim = static_cast<int>(X.size());
        Eigen::VectorXd grad(dim);

        Eigen::VectorXd X_plus = X;
        Eigen::VectorXd X_minus = X;

        // Abstract dimension-agnostic central difference gradient extraction wrapper
        for (int i = 0; i < dim; ++i) {
            X_plus[i] += h;
            X_minus[i] -= h;

            double f_plus = RvachevNd(X_plus, inter, m);
            double f_minus = RvachevNd(X_minus, inter, m);

            grad[i] = (f_plus - f_minus) / (2.0 * h);

            X_plus[i] = X[i];
            X_minus[i] = X[i];
        }

        return grad;
    }

    // =========================================================================
    // SPHERICAL WORKSPACE EMBEDDING & CANONICAL BASIS SETS (KRF FRAMEWORK)
    // =========================================================================

    void SphericalEvaluator::Init(unsigned int dimension) {
        N = dimension;
        ei.clear();
        n_ei.clear();
        ei_equatorial.clear();
        n_ei_equatorial.clear();

        // 1. Compile canonical bounding arrays (Structural Umbrella arcs)
        for (unsigned short i = 0; i < N; ++i) {
            Eigen::VectorXd tmp = Eigen::VectorXd::Zero(N);
            tmp[i] = 1.0; // Positive unit displacement bounds mapping Intersection
            Eigen::VectorXd ntmp = Eigen::VectorXd::Zero(N);
            ntmp[i] = -1.0; // Negative complement layout mapping Union conditions

            ei.push_back(tmp);
            n_ei.push_back(ntmp);
        }

        // 2. Generate focal axis anchoring coordinates centers (Barycentric simplex centers)
        northPole = Eigen::VectorXd::Constant(N, 1.0 / std::sqrt(N));
        southPole = -northPole;

        // 3. Map equatorial tracking structures via orthogonal hyperplane projection passes
        for (unsigned short i = 0; i < N; ++i) {
            // Projection rule: v_equat = e_i - (e_i . pole) * pole
            double dot = ei[i].dot(northPole);
            Eigen::VectorXd proj_i = (ei[i] - dot * northPole).normalized();
            ei_equatorial.push_back(proj_i);

            double dot_n = n_ei[i].dot(southPole);
            Eigen::VectorXd proj_ni = (n_ei[i] - dot_n * southPole).normalized();
            n_ei_equatorial.push_back(proj_ni);
        }
    }

    int SphericalEvaluator::getAngularSector(const Eigen::Vector3d& I, const std::vector<Eigen::VectorXd>& eq) {
        int n = static_cast<int>(eq.size());

        // Identify the localized sectoral segment containing the unit hypersphere coordinates projection
        for (int i = 0; i < n; ++i) {
            Eigen::Vector3d e_curr = eq[i].head<3>();
            Eigen::Vector3d e_next = eq[(i + 1) % n].head<3>();

            // Construct cross-product normal bounding separators intersecting the reference pole
            Eigen::Vector3d n1 = northPole.head<3>().cross(e_curr);
            Eigen::Vector3d n2 = e_next.cross(northPole.head<3>());

            // Dot-product check: concurrent alignment validates enclosure within the targeted sector bounds
            if (I.dot(n1) >= -1e-12 && I.dot(n2) >= -1e-12) {
                return i;
            }
        }
        return 0; // Fallback safety guard index
    }

    std::vector<Eigen::VectorXd> SphericalEvaluator::generateSphericalUmbrellaPoints(
        double t,
        const std::vector<Eigen::VectorXd>& directions,
        const Eigen::VectorXd& pole)
    {
        std::vector<Eigen::VectorXd> results;
        double tc = std::clamp(t, 0.0, 1.0);

        // Simulates dynamic simplex opening sequences mapping transition trajectories onto a unit profile shell
        for (const auto& d : directions) {
            results.push_back(((1.0 - tc) * d + tc * pole).normalized());
        }
        return results;
    }

    double SphericalEvaluator::ComputeHeight(
        const Eigen::VectorXd& I,
        const Eigen::VectorXd& ei,
        const Eigen::VectorXd& ek,
        const Eigen::VectorXd& n)
    {
        // Step 1: Construct primary localized relative vector segments in high-dimensional spaces
        // u1 tracks the chord direction separating canonical sector limits points
        Eigen::VectorXd u1 = (ei - ek).normalized();
        Eigen::VectorXd u3 = n; // Structural focal pole axis (pre-normalized)

        // Step 2: Establish the orthogonal basis framework via custom Gram-Schmidt projection loops
        Eigen::VectorXd u2 = ei + ek;
        u2 -= u2.dot(u1) * u1;
        u2 -= u2.dot(u3) * u3;
        u2.normalize();

        // Step 3: Project high-dimensional testing sample coordinate fields onto localized 3D frames
        Eigen::Vector3d I_proj;
        I_proj(0) = I.dot(u1);
        I_proj(1) = I.dot(u2);
        I_proj(2) = I.dot(u3);

        // Step 4: Map idealized 3D canonical coordinates anchors implicitly
        Eigen::Vector3d e1_local(1.0, 0.0, 0.0);
        Eigen::Vector3d e2_local(0.0, 1.0, 0.0);
        Eigen::Vector3d e3_local(0.0, 0.0, 1.0);

        // Step 5: Extract matrix determinants via triple scalar cross-products to solve the tangent aperture equation
        double A = I_proj.dot(e1_local.cross(e2_local));
        double C = I_proj.dot(e1_local.cross(e3_local));

        double denom = A * A + 4.0 * C * C;
        if (denom < 1e-15) return 0.0;

        // Step 6: Return the continuous positive height tracking component
        return std::sqrt(A * A / denom);
    }

    double SphericalEvaluator::Compute(bool isIntersection, const Eigen::VectorXd& X) {
        // 1. Magnitude calculation and zero-potential safety barrier checks
        double norm = X.norm();
        if (norm < 1e-15) return 0.0;

        // 2. Normalization phase: map input vector onto the unit space coordinate sphere point I
        Eigen::Vector3d I = X.normalized().head<3>();

        // 3. Extract parameter arrays tracking targeted composition constraints logic families
        const Eigen::Vector3d& localPole = isIntersection ? northPole : southPole;
        const std::vector<Eigen::VectorXd>& equatorialVectors = isIntersection ? ei_equatorial : n_ei_equatorial;

        // 4. Identify the active structural hemisphere quadrant using dot-product projections
        double dotProd = I.dot(localPole);
        double hemisphereSign = (dotProd >= 0.0 ? 1.0 : -1.0);

        // 5. Manage degenerate polar configuration singularities cleanly
        if (1.0 - std::abs(dotProd) < 1e-15) {
            return hemisphereSign * (isIntersection ? 1.0 : -1.0);
        }

        // 6. Trace the active angular sectoral domain enclosing point I
        int sector = getAngularSector(I, equatorialVectors);

        // 7. Core Solver call: compute continuous aperture height properties across the active sector limits
        double h = ComputeHeight(
            I,
            equatorialVectors[sector].head<3>(),
            equatorialVectors[(sector + 1) % 3].head<3>(),
            localPole
        );

        // 8. Re-apply polar directional orientation signs
        h *= hemisphereSign;

        // 9. Apply the metric barycentric normalization shift (offset = 1.0 / sqrt(3.0)) 
        // to finalize the directional generalized R-function potential field
        double invSqrtN = 1.0 / std::sqrt(3.0);
        double result = isIntersection ? (h - invSqrtN) : (invSqrtN - h);

        return result;
    }

} // namespace RCore