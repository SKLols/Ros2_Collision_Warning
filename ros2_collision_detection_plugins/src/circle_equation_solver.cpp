/**
 * @file circle_equation_solver.cpp
 * @author Michael Wittmann (miw2006@thi.de)
 * @brief Implementation of the Circle Equation Solver functions.
 * @version 0.1
 * @date 2022-05-23
 * 
 * @copyright Copyright (c) 2022
 * 
 * @author Sourabh Lolge (sol9471@thi.de)
 * @date 2024-11-12
 * @brief Updated from ROS 1 to ROS 2.
 * 
 */

#include "ros2_collision_detection/circle_equation_solver.hpp"
#include <rclcpp/rclcpp.hpp>
#include <gsl/gsl_errno.h>
#include <gsl/gsl_poly.h>

#define POLYNOMIAL_ARRAY_LENGTH 5   //!< quartic equation has variable of degree 0 to 4

namespace circle_equation_solver
{
    double computeCoefficientForPowerFour(double &accel_diff_sq_sin_adj, double &accel_diff_sq_cos_adj)
    {
        // [ (sin(alpha) * a_i - sin(beta) * a_j)^2 + (cos(alpha) * a_i - cos(beta) * a_j)^2 ] * 0.25
        return (accel_diff_sq_sin_adj + accel_diff_sq_cos_adj) / 4;
    }

    double computeCoefficientForPowerThree(double &accel_diff_sin_adj, double &accel_diff_cos_adj, double &speed_diff_sin_adj, double &speed_diff_cos_adj)
    {
        // (sin(alpha) * a_i - sin(beta) * a_j) * (sin(alpha) * v_i - sin(beta) * v_j) + 
        // (cos(alpha) * a_i - cos(beta) * a_j) * (cos(alpha) * v_i - cos(beta) * v_j)
        return accel_diff_sin_adj * speed_diff_sin_adj + accel_diff_cos_adj * speed_diff_cos_adj;
    }

    double computeCoefficientForPowerTwo(double &accel_diff_sin_adj, double &accel_diff_cos_adj, double &speed_diff_sq_sin_adj, double &speed_diff_sq_cos_adj, double &center_pos_x_diff, double &center_pos_y_diff)
    {
        // (sin(alpha) * v_i - sin(beta) * v_j)^2 + (sin(alpha) * a_i - sin(beta) * a_j) * (x_i - x_j) +
        // (cos(alpha) * v_i - cos(beta) * v_j)^2 + (cos(alpha) * a_i - cos(beta) * a_j) * (y_i - y_j)
        return speed_diff_sq_sin_adj + accel_diff_sin_adj * center_pos_x_diff + speed_diff_sq_cos_adj + accel_diff_cos_adj * center_pos_y_diff;
    }

    double computeCoefficientForPowerOne(double &speed_diff_sin_adj, double &speed_diff_cos_adj, double &center_pos_x_diff, double &center_pos_y_diff)
    {
        // [ (sin(alpha) * v_i - sin(beta) * v_j) * (x_i - x_j) +
        //   (cos(alpha) * v_i - cos(beta) * v_j) * (y_i - y_j)  ] * 2
        return (speed_diff_sin_adj * center_pos_x_diff + speed_diff_cos_adj * center_pos_y_diff) * 2;
    }

    double computeCoefficientForPowerZero(double &center_pos_x_diff_sq, double &center_pos_y_diff_sq, double &radius_sum_sq)
    {
        // (x_i - x_j)^2 + (y_i - y_j)^2 - (r_i + r_j)^2
        return center_pos_x_diff_sq + center_pos_y_diff_sq - radius_sum_sq;
    }

    int getHighestPolynomialNonZeroDegree(std::array<double, 5> &coefficients)
    {
        // polynomial coefficients are stored from lower degree to higher degree
        int highest_non_zero_degree = 4;    //!< quartic equation
        for(int i = 0; i < POLYNOMIAL_ARRAY_LENGTH - 1; i++)
        {
            if(coefficients.at(POLYNOMIAL_ARRAY_LENGTH - 1 - i) == 0)
            {
                highest_non_zero_degree--;
            }
            else
            {
                break;
            } 
        }

        // log highest non-zero polynomial degree
        RCLCPP_DEBUG(rclcpp::get_logger("circle_equation_solver"), "Highest polynomial degree: %d", highest_non_zero_degree);

        return highest_non_zero_degree;
    }

    std::vector<double> solvePolynomialEquationGSL(std::array<double, POLYNOMIAL_ARRAY_LENGTH> &coefficients)
    {
        // the real positive roots to be returned
        std::vector<double> real_positive_roots;

        // log coefficients
        RCLCPP_DEBUG(rclcpp::get_logger("circle_equation_solver"), "Computed coefficients:");
        for(int i = 0; i < POLYNOMIAL_ARRAY_LENGTH; i++)
        {
            RCLCPP_DEBUG(rclcpp::get_logger("circle_equation_solver"), "coefficient %d = %f", i, coefficients.at(i));
        }

        // get the highest degree of the polynomial coefficient that is not zero
        int highest_polynomial_degree_nzero = getHighestPolynomialNonZeroDegree(coefficients);
        if(highest_polynomial_degree_nzero < 1)
        {
            // all polynomials of degree >= 1 are zero
            // no solution possible for P(t) = b_0 * t^0
            // return empty real_positive_roots vector
            RCLCPP_ERROR(rclcpp::get_logger("circle_equation_solver"), "Polynomial degree is too low!");
            return real_positive_roots;
        }

        int polynomial_array_length_adjusted = highest_polynomial_degree_nzero + 1; //!< adjusted length of the polynomial coefficient array for GSL

        // prepare array of coefficients for GSL
        double poly_coefficients[polynomial_array_length_adjusted];
        for (int i = 0; i < polynomial_array_length_adjusted; i++)
        {
            poly_coefficients[i] = coefficients.at(i);
        }

        // array stores results of GSL, alternating real and imaginary components
        double complex_results[2*(polynomial_array_length_adjusted - 1)];

        // Workspace necessary for GSL to solve the polynomial equation
        gsl_poly_complex_workspace *gsl_workspace = gsl_poly_complex_workspace_alloc(polynomial_array_length_adjusted);

        // solve the polynomial equation defined by poly_coefficients and store results to array complex_results
        int status = gsl_poly_complex_solve(poly_coefficients, polynomial_array_length_adjusted, gsl_workspace, complex_results);

        // free GSL workspace
        gsl_poly_complex_workspace_free(gsl_workspace);

        if(status == GSL_EFAILED)
        {
            // TODO: handle GSL error
            RCLCPP_ERROR(rclcpp::get_logger("circle_equation_solver"), "GSL error");
        }

        // log results from GSL
        RCLCPP_DEBUG(rclcpp::get_logger("circle_equation_solver"), "Results from GSL:");
        for(int i = 0; i < polynomial_array_length_adjusted; i++)
        {
            RCLCPP_DEBUG(rclcpp::get_logger("circle_equation_solver"), "result %d | real = %+.18f | imag = %+.18f", i, complex_results[2*i], complex_results[2*i+1]);
        }

        // populate result vector only with real roots
        for (int i = 0; i < polynomial_array_length_adjusted; i++)
        {
            if(complex_results[2*i+1] == 0)   // check if imaginary part is zero --> root is real
            {
                RCLCPP_DEBUG(rclcpp::get_logger("circle_equation_solver"), "result %d is real: %+.18f %+.18f", i, complex_results[2*i], complex_results[2*i+1]);

                if (complex_results[2*i] > 0) // check if real root is positive
                {
                    // only real positive roots are returned
                    real_positive_roots.push_back(complex_results[2*i]);
                    RCLCPP_DEBUG(rclcpp::get_logger("circle_equation_solver"), "result %d is real and positive: %+.18f %+.18f", i, complex_results[2*i], complex_results[2*i+1]);
                }
            }
            else
            {
                RCLCPP_DEBUG(rclcpp::get_logger("circle_equation_solver"), "result %d is complex: %+.18f %+.18f", i, complex_results[2*i], complex_results[2*i+1]);
            }
        }

        return real_positive_roots;   
    }
}