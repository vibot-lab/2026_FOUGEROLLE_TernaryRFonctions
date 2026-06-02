/**
 * @file ImplicitObject.h
 * @author Yohan FOUGEROLLE
 * @date 2026
 * @brief This file is part of a research project on Generalized R-functions
 * for N-dimensional implicit modeling.
 */

#pragma once

#include <Eigen/Dense>
#include <vector>
#include <memory>
#include <algorithm>
#include <cmath>


#ifndef M_PI
#define _USE_MATH_DEFINES
#include <cmath>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#endif

 /**
  * Base class for N-Dimensional implicit objects.
  */
class ImplicitObject {
public:
    virtual ~ImplicitObject() = default;

    // Evaluation in 3D
    virtual double Evaluate(const Eigen::Vector3d& P) const = 0;

    // Gradient signature matching the base framework requirements
    virtual Eigen::VectorXd Gradient(const Eigen::VectorXd& P, double h = 1e-7) const = 0;
};

/**
 * N-Dimensional Sphere.
 */
class ImplicitSphere : public ImplicitObject {
private:
    Eigen::Vector3d m_center;
    double m_radius;

public:
    ImplicitSphere(const Eigen::Vector3d& center, double radius)
        : m_center(center), m_radius(radius) {
    }

    double Evaluate(const Eigen::Vector3d& P) const override {
        return m_radius - (P - m_center).norm();
    }

    Eigen::VectorXd Gradient(const Eigen::VectorXd& P, double h = 1e-7) const override {
        Eigen::Vector3d P3 = P.head<3>();
        Eigen::Vector3d grad = (m_center - P3).normalized();
        return grad;
    }

    const Eigen::Vector3d& getCenter() const { return m_center; }
    void setCenter(const Eigen::Vector3d& center) { m_center = center; }
    double getRadius() const { return m_radius; }
    void setRadius(double radius) { m_radius = radius; }
};

/**
 * N-Dimensional Hyperplane defined by a segment in the viewing plane.
 */
class ImplicitPlane : public ImplicitObject {
private:
    Eigen::Vector3d m_normal;
    double m_offset;
    Eigen::Vector3d m_pStart, m_pEnd;

    void updateGeometry() {
        Eigen::Vector3d dir = m_pEnd - m_pStart;
        m_normal = -dir.cross(Eigen::Vector3d::UnitZ()).normalized();
        m_offset = -m_normal.dot(m_pStart);
    }

public:
    ImplicitPlane(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2)
        : m_pStart(p1), m_pEnd(p2) {
        updateGeometry();
    }

    double Evaluate(const Eigen::Vector3d& P) const override {
        return m_normal.dot(P) + m_offset;
    }

    Eigen::VectorXd Gradient(const Eigen::VectorXd& P, double h = 1e-7) const override {
        return m_normal;
    }

    void setPoints(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2) {
        m_pStart = p1; m_pEnd = p2; updateGeometry();
    }

    const Eigen::Vector3d& getP1() const { return m_pStart; }
    const Eigen::Vector3d& getP2() const { return m_pEnd; }
    const Eigen::Vector3d& getNormal() const { return m_normal; }
};

/**
 * Data structure for research and debugging.
 */
struct VertexResearchData {
    double f_cyl[3];
    double f_bin[3];
    double f_tern;
    Eigen::Vector3d g_cyl[3];
    Eigen::Vector3d g_bin[3];
    Eigen::Vector3d g_tern;
    int id_cyl;
};

/**
 * Cylinder implementation using  R-functions.
 */
class ImplicitCylinder : public ImplicitObject {
private:
    Eigen::Vector3d m_p1;
    Eigen::Vector3d m_p2;
    double m_radius;

public:
    ImplicitCylinder(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, double radius)
        : m_p1(p1), m_p2(p2), m_radius(radius) {
    }

    double Evaluate(const Eigen::Vector3d& P) const override {
        Eigen::Vector3d AB = m_p2 - m_p1;
        double L = AB.norm();
        if (L < 1e-12) return m_radius - (P - m_p1).norm();

        Eigen::Vector3d u = AB / L;
        Eigen::Vector3d AP = P - m_p1;
        double w = AP.dot(u);
        double d_axis = (AP - w * u).norm();

        double f_rad = m_radius - d_axis;
        //  tubes
        //double f_rad = 0.1 - abs(m_radius - d_axis);



        double f_p1 = w;
        double f_p2 = L - w;

        return SmoothIntersection(f_rad, SmoothIntersection(f_p1, f_p2));
    }

    double SmoothIntersection(double a, double b) const {
        return a + b - std::sqrt(a * a + b * b);
    }

    Eigen::VectorXd Gradient(const Eigen::VectorXd& P, double h = 1e-7) const override {
        return GradientAnalytique(P.head<3>());
    }

    Eigen::Vector3d GradientAnalytique(const Eigen::Vector3d& P) const {
        Eigen::Vector3d AB = m_p2 - m_p1;
        double L = AB.norm();
        if (L < 1e-12) return (m_p1 - P).normalized();

        Eigen::Vector3d u = AB / L;
        Eigen::Vector3d AP = P - m_p1;
        double w = AP.dot(u);

        Eigen::Vector3d radialVec = AP - w * u; // Vecteur de l'axe vers P
        double d_axis = radialVec.norm();

        // On recalcule les potentiels pour savoir lequel "pilote" la surface à cet endroit
        double f_rad = m_radius - d_axis;
        double f_p1 = w;
        double f_p2 = L - w;

        // Logique de sélection basée sur le "Min" (le plus contraignant)
        // C'est l'approximation C0 du gradient de ta SmoothIntersection.
        if (f_rad <= f_p1 && f_rad <= f_p2) {
            // Zone latérale : le gradient pointe vers l'axe (sens croissant de f_rad)
            return (d_axis > 1e-12) ? (Eigen::Vector3d)(-radialVec / d_axis) : Eigen::Vector3d::Zero();
        }
        else if (f_p1 <= f_p2) {
            // Bouchon P1 : le potentiel w augmente selon l'axe u
            return u;
        }
        else {
            // Bouchon P2 : le potentiel (L-w) augmente en sens inverse de u
            return -u;
        }
    }

    // Accessors
    void setPoints(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2) { m_p1 = p1; m_p2 = p2; }
    void setRadius(double r) { m_radius = r; }

    /**
     * Mesh generation for the cylinder.
     */
    std::pair<std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3i>>
        MakeTube(unsigned int radialSampling = 40, unsigned int axialSampling = 5) {
        std::vector<Eigen::Vector3d> localVertices;
        std::vector<Eigen::Vector3i> localFaces;

        if (radialSampling < 3 || axialSampling < 1) return { localVertices, localFaces };

        Eigen::Vector3d axis = m_p2 - m_p1;
        double length = axis.norm();
        if (length < 1e-12) return { localVertices, localFaces };
        Eigen::Vector3d dir = axis / length;

        Eigen::Vector3d ref(0, 1, 0);
        if (std::abs(dir.dot(ref)) > 0.99) ref = Eigen::Vector3d(0, 0, 1);
        Eigen::Vector3d U = dir.cross(ref).normalized();
        Eigen::Vector3d V = dir.cross(U).normalized();

        Eigen::Vector3d deltaStep = axis / (double)(axialSampling);

        for (unsigned int i = 0; i <= axialSampling; ++i) {
            Eigen::Vector3d center = m_p1 + (double)i * deltaStep;
            for (unsigned int j = 0; j < radialSampling; ++j) {
                double angle = (double)j * (2.0 * EIGEN_PI / (double)radialSampling);
                localVertices.push_back(center + (m_radius * std::cos(angle)) * U + (m_radius * std::sin(angle)) * V);
            }
        }

        for (unsigned int i = 0; i < axialSampling; ++i) {
            int currRing = i * radialSampling;
            int nextRing = (i + 1) * radialSampling;
            for (unsigned int j = 0; j < radialSampling; ++j) {
                int next_j = (j + 1) % radialSampling;
                int v0 = currRing + j;
                int v1 = currRing + next_j;
                int v2 = nextRing + j;
                int v3 = nextRing + next_j;
                localFaces.push_back(Eigen::Vector3i(v0, v1, v3));
                localFaces.push_back(Eigen::Vector3i(v0, v3, v2));
            }
        }
        return { localVertices, localFaces };
    }
};