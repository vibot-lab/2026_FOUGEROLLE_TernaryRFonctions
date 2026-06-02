
/**
 * @file CAD_Scene.h
 * @author Yohan FOUGEROLLE
 * @date 2026
 * @brief This file is part of a research project on Generalized R-functions
 * for N-dimensional implicit modeling.
 * @details Licensed under the MIT License.
 */

#pragma once


#include <string>
#include <vector>
#include <memory>
#include <cmath>
#include <Eigen/Core>
#include "ImplicitObjects.h"
#include "Rfunctions.h"


 /**
  * @struct BenchResult
  * @brief Storage capsule capturing runtime metrics across the chrono-isolated extraction pipeline.
  */
struct BenchResult {
    std::string logicName;  ///< Label identification string of the composition family evaluated
    int resolution;         ///< Spatial voxel grid discretization density dimension index
    double totalTime_ms;    ///< Cumulative end-to-end processing pipeline execution cost
    double generationTime;  ///< Sub-computation duration isolated for the raw Marching Cubes pass (ms)
    double cleanupTime;     ///< Sub-computation duration allocated for topological optimization passes (ms)
    int finalFaceCount;     ///< Total amount of validated manifold triangular face primitives generated
    double meanError;       ///< Arithmetic field residual mean: average of |Evaluate(P)|
    double maxError;        ///< Maximum L-infinity field divergence boundary tracked on the isosurface
};

/**
 * @class CAD_Scene
 * @brief Concrete composite implicit shape container mapping primitive objects onto R-function logic trees.
 * @details Inherits polymorphically from ImplicitObject to provide unified field evaluation and analytical
 * spatial gradient estimation across overlapping multi-primitive configurations.
 */
class CAD_Scene : public ImplicitObject {
public:
    // =========================================================================
    // MEMBERS & COUPLING OPERATORS BOUNDS
    // =========================================================================

    std::vector<ImplicitObject*> primitives;            ///< Polymorphic tracking containment array holding analytical inputs
    bool isIntersection;                                ///< System toggle flag configuration: true = Intersection, false = Union

    RCore::SphericalEvaluator* TernarySphericalOperator = nullptr; ///< Reference address binding the global ternary framework

    LogicType currentLogic = LogicType::TERNARY_KRF;    ///< Currently selected algebraic field composition mode
    double p_parameter = 2.0;                           ///< Bounded regularization order exponent mapped to Rp formulations

    // =========================================================================
    // LIFECYCLE MANAGEMENT & VALUE SETTERS/GETTERS
    // =========================================================================

    /**
     * @brief Explicit structure constructor mapping base operation assignments.
     */
    CAD_Scene(bool inter) : isIntersection(inter) {}

    void setLogicalOperation(bool isInter) { isIntersection = isInter; }
    bool getLogicalOperation() const { return isIntersection; }

    // =========================================================================
    // POLYMORPHIC SCALAR FIELD EVALUATIONS
    // =========================================================================

    /**
     * @brief Computes the composite global scalar potential at target coordinate space point P.
     * @details Drives sequential field evaluations across all linked sub-primitives,
     * channeling outputs into the chosen algebraic logical operator mapping profile.
     * @param P Spatial 3D sampling coordinate point location vector.
     * @return double The resulting regularized implicit potential scalar metric value.
     */
    double Evaluate(const Eigen::Vector3d& P) const override {
        if (primitives.empty()) return -1.0;

        // 1. Gather isolated potential fields across the entire primitive collection
        Eigen::VectorXd X(primitives.size());
        for (int i = 0; i < (int)primitives.size(); ++i) {
            X[i] = primitives[i]->Evaluate(P);
        }

        // 2. Map potentials utilizing the targeted operational composition family
        switch (currentLogic) {
        case LogicType::MIN_NAIVE:
        {
            //uncomment to check if the correct function is called
            //std::cout << "MAX NAIVE" << std::endl;
            //system("pause");
            if(isIntersection)
                return X.minCoeff();
            else
                return X.maxCoeff(); // Standard non-differentiable boundary selection matching Intersection logic
        }
        case LogicType::TERNARY_KRF:
        {
            //uncomment to check if the correct function is called
            // std::cout << "TERNARY" << std::endl;
            //system("pause");
            return TernarySphericalOperator->ComputeNormalized(isIntersection, X); // Directional regularized KRF path
        }
        case LogicType::BINARY_RP:
        {
            //uncomment to check if the correct function is called
            //std::cout << "COMBINATORIAL RP" << std::endl;
            //system("pause");
            // Inductive permutation averaging tree: evaluates symmetric combinations to counteract non-associativity bias
            double resBC = RCore::Rp(X[1], X[2], false);
            resBC = RCore::Rp(X[0], resBC, false);

            double resAC = RCore::Rp(X[0], X[2], false);
            resAC = RCore::Rp(X[1], resAC, false);

            double resAB = RCore::Rp(X[0], X[1], false);
            resAB = RCore::Rp(X[2], resAB, false);

            double res = (resAB + resAC + resBC) / 3.0;

            // -----------------------------------------------------------------
            // CRITICAL SAFETY GUARD: ARTIFACT SHIELD & NaN PURGE
            // -----------------------------------------------------------------
            // Numerical safety block: handles arithmetic singular instability or indeterminate conditions. 
            // Forces a strong exterior potential value, warning the Marching Cubes sampler to discard the cell region.
            if (std::isnan(res)) return 1.0;

            return res;
        }
        }
        return -1.0;
    }

    // =========================================================================
    // DIFFERENTIAL SOLVERS & GRADIENT EXTRACTIONS
    // =========================================================================

    /**
     * @brief Computes the 3D spatial gradient vector field at explicit domain position vectors.
     * @details Evaluates directional variations using an O(h^2) central finite difference scheme
     * applied onto the continuous global multi-primitive composition profile.
     * @param P Coordinate query input position vector.
     * @param h Differentiation incremental step size used for numerical perturbation loops.
     * @return Eigen::VectorXd The estimated 3D spatial directional gradient vector array.
     */
    Eigen::VectorXd Gradient(const Eigen::VectorXd& P, double h = 1e-7) const override {
        if (primitives.empty()) return Eigen::VectorXd::Zero(3);

        // Extract native spatial 3D cartesian coordinates parameters
        Eigen::Vector3d P3 = P.head<3>();
        Eigen::Vector3d grad;

        // Perform localized symmetric perturbations along orthogonal axis channels
        for (int i = 0; i < 3; ++i) {
            Eigen::Vector3d P_plus = P3;
            Eigen::Vector3d P_moins = P3;

            P_plus[i] += h;
            P_moins[i] -= h;

            // Evaluate perturbed positions. Under TERNARY_KRF, this invokes 
            // the continuous ComputeNormalized engine where field smoothness guarantees gradient stability.
            double f_plus = this->Evaluate(P_plus);
            double f_moins = this->Evaluate(P_moins);

            // Compute central partial spatial derivative component
            grad[i] = (f_plus - f_moins) / (2.0 * h);
        }

        return grad;
    }
};