#include "ros2_collision_detection/ttc_algorithm.hpp"
//#include "ros2_collision_detection/circle_algorithm.hpp"
#include "ros2_collision_detection/n_circle_algorithm.hpp"
#include <cmath>
#include <rclcpp/rclcpp.hpp>


#define POLYNOMIAL_ARRAY_LENGTH 5   //!< quartic equation has variable of degree 0 to 4

namespace ros2_collision_detection_plugins {

    // Base class to handle node initialization
    class NodeInitializer {
    public:
        NodeInitializer() {
            // Generate a unique node name dynamically using the current time
            node_ = std::make_shared<rclcpp::Node>("dummy_node_" + std::to_string(std::chrono::system_clock::now().time_since_epoch().count()));
            RCLCPP_INFO(node_->get_logger(), "Node initializer constructor called");
            
            //node_ = std::make_shared<rclcpp::Node>("dummy_node");
            //printf("Node initializer constructor called\n");
        }

        rclcpp::Node::SharedPtr getNode() {
            return node_;
        }

    protected:
        rclcpp::Node::SharedPtr node_;
    };

    class CircleAlgorithm : public ros2_collision_detection::TTCAlgorithm, public NodeInitializer
    {
        public:
            CircleAlgorithm()
            
            {
                //RCLCPP_INFO(node_->get_logger(),"CircleAlgorithm constructed");
                RCLCPP_INFO(node_->get_logger(), "CircleAlgorithm constructor called");
                printf("Constructor created");
            }

            void initialize(double side_length) override 
            {
                side_length_ = side_length;
            }
            //void init(parameter_map_t &parameter_map) override
            //{
            //    parameter_map_=parameter_map
            //}

            double area() override 
            {
                return side_length_*side_length_;
            }

            double computeCoefficientForPowerFour(double &accel_diff_sq_sin_adj, double &accel_diff_sq_cos_adj)
            {
                return (accel_diff_sq_sin_adj + accel_diff_sq_cos_adj) / 4;
            }

            protected:
                double side_length_;
                
            //std::optional<double> calculateTTC(
            //const object_motion_t &subject_object_motion,
            //const object_motion_t &perceived_object_motion
            //) override
            //{
            //    return 
            //}
    };

    class CircleEquationSolver : public ros2_collision_detection::TTCAlgorithm, public NodeInitializer //Just change class name as per header file
    {
        public:
            CircleEquationSolver() //Just change to class name as per header file

            //explicit CircleEquationSolver(const rclcpp::Node::SharedPtr &node) : node_(node) {
            //    printf("Constructor 2 created\n");
            //}

            {
                // Create a dummy node or handle the lack of a real node here
                //rclcpp::Node::SharedPtr dummy_node = std::make_shared<rclcpp::Node>("dummy_node");
                //node_ = dummy_node;
                //RCLCPP_INFO(node_->get_logger(),"CircleEquationSolver constructed");
                RCLCPP_INFO(node_->get_logger(), "CircleEquationSolver constructor called");
                //printf("Constructor 2 created");
            }

            double computeCoefficientForPowerFour(double &accel_diff_sq_sin_adj, double &accel_diff_sq_cos_adj)
            {
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

                // log highest non-zero polynomial degree //
                RCLCPP_DEBUG(node_->get_logger(),"circle_equation_solver::getHighestPolynomialNonZeroDegree: Highest polynomial degree: %d",highest_non_zero_degree);
                printf("circle_equation_solver::getHighestPolynomialNonZeroDegree: Highest polynomial degree: %d\n", highest_non_zero_degree);

                return highest_non_zero_degree;
            }

            std::vector<double> solvePolynomialEquationGSL(std::array<double, POLYNOMIAL_ARRAY_LENGTH> &coefficients)
            {
                printf("Test");

                // the real positive roots to be returned
                std::vector<double> real_positive_roots;

                // log coefficients
                RCLCPP_DEBUG(node_->get_logger(),"circle_equation_solver::solvePolynomialEquationGSL: computed coefficients:");
                for(int i = 0; i < POLYNOMIAL_ARRAY_LENGTH; i++)
                {
                    RCLCPP_DEBUG(node_->get_logger(),"coefficient %d = %f", i, coefficients.at(i));
                }

                // get the highest degree of the polynomial coefficient that is not zero
                int highest_polynomial_degree_nzero = getHighestPolynomialNonZeroDegree(coefficients);
                if(highest_polynomial_degree_nzero < 1)
                {
                    // all polynomials of degree >= 1 are zero
                    // no solution possible for P(t) = b_0 * t^0
                    // return empty real_positive_roots vector
                    RCLCPP_ERROR(node_->get_logger(),"circle_equation_solver::solvePolynomialEquationGSL: polynomial degree is too low!");
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
                    RCLCPP_ERROR(node_->get_logger(),"circle_equation_solver::solvePolynomialEquationGSL: GSL error");
                }

                // log results from GSL
                RCLCPP_DEBUG(node_->get_logger(),"circle_equation_solver::solvePolynomialEquationGSL: results from GSL: ");
                for(int i = 0; i < polynomial_array_length_adjusted; i++)
                {
                    RCLCPP_DEBUG(node_->get_logger(),"result %d | real = %+.18f | imag = %+.18f",i, complex_results[2*i], complex_results[2*i+1]);
                }

                // populate result vector only with real roots
                for (int i = 0; i < polynomial_array_length_adjusted; i++)
                {
                    // TODO: might need approximate check instead of exact check
                    if(complex_results[2*i+1] == 0)   // check if imaginary part is zero --> root is real
                    {
                        RCLCPP_DEBUG(node_->get_logger(),"circle_equation_solver::solvePolynomialEquationGSL: result %d is real: %+.18f %+.18f", i, complex_results[2*i], complex_results[2*i+1]);

                        if (complex_results[2*i] > 0) // check if real root is positive
                        {
                            // only real positive roots are returned
                            real_positive_roots.push_back(complex_results[2*i]);
                            RCLCPP_DEBUG(node_->get_logger(),"circle_equation_solver::solvePolynomialEquationGSL: result %d is real and positive: %+.18f %+.18f", i, complex_results[2*i], complex_results[2*i+1]);
                        }
                    }
                    else
                    {
                        RCLCPP_DEBUG(node_->get_logger(),"circle_equation_solver::solvePolynomialEquationGSL: result %d is complex: %+.18f %+.18f", i, complex_results[2*i], complex_results[2*i+1]);
                    }
                }

                return real_positive_roots; 


            }

            //Below part should be deleted later it was just sample for plugin working
            void initialize(double side_length) override 
            {
                side_length_ = side_length;
            }
            //void init(parameter_map_t &parameter_map) override
            //{
            //    parameter_map_=parameter_map
            //}

            double area() override 
            {
                return side_length_*side_length_*side_length_;
            }

            protected:
                double side_length_;

            
    };

} // namespace ros2_collision_detection_plugins


#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(ros2_collision_detection_plugins::CircleAlgorithm, ros2_collision_detection::TTCAlgorithm)
PLUGINLIB_EXPORT_CLASS(ros2_collision_detection_plugins::CircleEquationSolver, ros2_collision_detection::TTCAlgorithm)


