#include "ros2_collision_detection/ttc_algorithm.hpp"
//#include "ros2_collision_detection/circle_algorithm.hpp"
#include "ros2_collision_detection/n_circle_algorithm.hpp"
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <optional>
#include <variant>

#define DEFAULT_CIRCLE_COUNT 1      //!< default number of circles if no circle_count is passed
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

    class CircleEquationSolver : /*public ros2_collision_detection::TTCAlgorithm,*/ public NodeInitializer //Just change class name as per header file
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
                RCLCPP_DEBUG(node_->get_logger(),"CircleEquationSolver::getHighestPolynomialNonZeroDegree: Highest polynomial degree: %d",highest_non_zero_degree);
                printf("CircleEquationSolver::getHighestPolynomialNonZeroDegree: Highest polynomial degree: %d\n", highest_non_zero_degree);

                return highest_non_zero_degree;
            }

            std::vector<double> solvePolynomialEquationGSL(std::array<double, POLYNOMIAL_ARRAY_LENGTH> &coefficients)
            {
                printf("Test");

                // the real positive roots to be returned
                std::vector<double> real_positive_roots;

                // log coefficients
                RCLCPP_DEBUG(node_->get_logger(),"CircleEquationSolver::solvePolynomialEquationGSL: computed coefficients:");
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
                    RCLCPP_ERROR(node_->get_logger(),"CircleEquationSolver::solvePolynomialEquationGSL: polynomial degree is too low!");
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
                    RCLCPP_ERROR(node_->get_logger(),"CircleEquationSolver::solvePolynomialEquationGSL: GSL error");
                }

                // log results from GSL
                RCLCPP_DEBUG(node_->get_logger(),"CircleEquationSolver::solvePolynomialEquationGSL: results from GSL: ");
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
                        RCLCPP_DEBUG(node_->get_logger(),"CircleEquationSolver::solvePolynomialEquationGSL: result %d is real: %+.18f %+.18f", i, complex_results[2*i], complex_results[2*i+1]);

                        if (complex_results[2*i] > 0) // check if real root is positive
                        {
                            // only real positive roots are returned
                            real_positive_roots.push_back(complex_results[2*i]);
                            RCLCPP_DEBUG(node_->get_logger(),"CircleEquationSolver::solvePolynomialEquationGSL: result %d is real and positive: %+.18f %+.18f", i, complex_results[2*i], complex_results[2*i+1]);
                        }
                    }
                    else
                    {
                        RCLCPP_DEBUG(node_->get_logger(),"CircleEquationSolver::solvePolynomialEquationGSL: result %d is complex: %+.18f %+.18f", i, complex_results[2*i], complex_results[2*i+1]);
                    }
                }

                return real_positive_roots; 


            }

            //Below part should be deleted later it was just sample for plugin working
            //void initialize(double side_length) override 
            //{
            //    side_length_ = side_length;
            //}
            //void init(parameter_map_t &parameter_map) override
            //{
            //    parameter_map_=parameter_map
            //}

            //double area() override 
            //{
            //    return side_length_*side_length_*side_length_;
            //}

            //protected:
            //    double side_length_;

            
    };

    class CircleAlgorithm : public ros2_collision_detection::TTCAlgorithm, public NodeInitializer
    {
        public:
            CircleAlgorithm()
            
            {
                RCLCPP_INFO(node_->get_logger(), "CircleAlgorithm constructed");
            }

            std::string convertMotionStructToString(const ros2_collision_detection::object_motion_t& object_motion)
            {
            	std::stringstream result;

                result << "center_pos_x = " << object_motion.center_pos_x << std::endl;
            	result << "center_pos_y = " << object_motion.center_pos_y << std::endl;
            	result << "length = " << object_motion.length << std::endl;
            	result << "width = " << object_motion.width << std::endl;
            	result << "heading = " << object_motion.heading << std::endl;
            	result << "speed = " << object_motion.speed << std::endl;
            	result << "acceleration = " << object_motion.acceleration << std::endl;

                return result.str();
            }

            double computeSinFromHeading(const float &heading)
            {
                double result = sin(heading * M_PI / 180.0);
                return result;
            }

            double computeCosFromHeading(const float &heading)
            {
                double result = cos(heading * M_PI / 180.0);
                return result;
            }

            double computeHeadingAdjustedValue(const float &value_to_adjust, const double &trigonometric_value)
            {
                return trigonometric_value * value_to_adjust;
            }

            double computeRadiusFromLength(const float &length, const float &width)
            {
                // radius = 0.5 * sqrt((length)^2 + (width)^2)
                return sqrt(length * length + width * width) / 2;
            }

            void initialize(ros2_collision_detection::parameter_map_t &parameter_map) override
            {
                RCLCPP_INFO(rclcpp::get_logger("CircleAlgorithm"), "CircleAlgorithm does  initialize.");
            }

            std::optional<double> calculateTTC(const ros2_collision_detection::object_motion_t &subject_object_motion, const ros2_collision_detection::object_motion_t &perceived_object_motion)
            {   
                // the Time-To-Collision optional return value
                std::optional<double> ttc_optional;

                // log the received object motions
                RCLCPP_DEBUG_STREAM(node_->get_logger(),"subject object motion: \n" << convertMotionStructToString(subject_object_motion));
                RCLCPP_DEBUG_STREAM(node_->get_logger(),"perceived object motion: \n" << convertMotionStructToString(perceived_object_motion));

                if(subject_object_motion.length <= 0 || subject_object_motion.width <= 0 || perceived_object_motion.length <= 0 || perceived_object_motion.width <= 0)
                {
                    RCLCPP_ERROR(node_->get_logger(),"CircleAlgorithm::calculateTTC: length or width is not allowed to be zero or lower.");
                    return ttc_optional;
                }

                if(subject_object_motion.heading < 0 || subject_object_motion.heading > 360 || perceived_object_motion.heading < 0 || perceived_object_motion.heading > 360)
                {
                    RCLCPP_ERROR(node_->get_logger(),"CircleAlgorithm::calculateTTC: heading value must be in range [0; 360].");
                    return ttc_optional;
                }

                double sin_subject_obj_heading = computeSinFromHeading(subject_object_motion.heading);      //!< sin(alpha)
                double cos_subject_obj_heading = computeCosFromHeading(subject_object_motion.heading);      //!< cos(alpha)
                double sin_perceived_obj_heading = computeSinFromHeading(perceived_object_motion.heading);  //!< sin(beta)
                double cos_perceived_obj_heading = computeCosFromHeading(perceived_object_motion.heading);  //!< cos(beta)

                double accel_subject_obj_sin_adjusted = computeHeadingAdjustedValue(subject_object_motion.acceleration, sin_subject_obj_heading);       //!< sin(alpha) * a_i
                double accel_subject_obj_cos_adjusted = computeHeadingAdjustedValue(subject_object_motion.acceleration, cos_subject_obj_heading);       //!< cos(alpha) * a_i
                double accel_perceived_obj_sin_adjusted = computeHeadingAdjustedValue(perceived_object_motion.acceleration, sin_perceived_obj_heading); //!< sin(beta) * a_j
                double accel_perceived_obj_cos_adjusted = computeHeadingAdjustedValue(perceived_object_motion.acceleration, cos_perceived_obj_heading); //!< cos(beta) * a_j

                double speed_subject_obj_sin_adjusted = computeHeadingAdjustedValue(subject_object_motion.speed, sin_subject_obj_heading);          //!< sin(alpha) * v_i
                double speed_subject_obj_cos_adjusted = computeHeadingAdjustedValue(subject_object_motion.speed, cos_subject_obj_heading);          //!< cos(alpha) * v_i
                double speed_perceived_obj_sin_adjusted = computeHeadingAdjustedValue(perceived_object_motion.speed, sin_perceived_obj_heading);    //!< sin(beta) * v_j
                double speed_perceived_obj_cos_adjusted = computeHeadingAdjustedValue(perceived_object_motion.speed, cos_perceived_obj_heading);    //!< cos(beta) * v_j

                double radius_subject_obj = computeRadiusFromLength(subject_object_motion.length, subject_object_motion.width);        //!< r_i
                double radius_perceived_obj = computeRadiusFromLength(perceived_object_motion.length, perceived_object_motion.width);  //!< r_j

                // differences between subject object's values and perceived object's values
                double accel_diff_sin_adjusted = accel_subject_obj_sin_adjusted - accel_perceived_obj_sin_adjusted; //!< sin(alpha) * a_i - sin(beta) * a_j
                double accel_diff_cos_adjusted = accel_subject_obj_cos_adjusted - accel_perceived_obj_cos_adjusted; //!< cos(alpha) * a_i - cos(beta) * a_j
                double speed_diff_sin_adjusted = speed_subject_obj_sin_adjusted - speed_perceived_obj_sin_adjusted; //!< sin(alpha) * v_i - sin(beta) * v_j
                double speed_diff_cos_adjusted = speed_subject_obj_cos_adjusted - speed_perceived_obj_cos_adjusted; //!< cos(alpha) * v_i - cos(beta) * v_j
                double center_pos_x_diff = subject_object_motion.center_pos_x - perceived_object_motion.center_pos_x;   //!< x_i - x_j
                double center_pos_y_diff = subject_object_motion.center_pos_y - perceived_object_motion.center_pos_y;   //!< y_i - y_j

                // sum of the radii of subject object and perceived object
                double radius_sum = radius_subject_obj + radius_perceived_obj;  //!< r_i + r_j

                // squares of the differences
                double accel_diff_square_sin_adjusted = accel_diff_sin_adjusted * accel_diff_sin_adjusted;  //!< (sin(alpha) * a_i - sin(beta) * a_j)^2
                double accel_diff_square_cos_adjusted = accel_diff_cos_adjusted * accel_diff_cos_adjusted;  //!< (cos(alpha) * a_i - cos(beta) * a_j)^2
                double speed_diff_square_sin_adjusted = speed_diff_sin_adjusted * speed_diff_sin_adjusted;  //!< (sin(alpha) * v_i - sin(beta) * v_j)^2
                double speed_diff_square_cos_adjusted = speed_diff_cos_adjusted * speed_diff_cos_adjusted;  //!< (cos(alpha) * v_i - cos(beta) * v_j)^2
                double center_pos_x_diff_square = center_pos_x_diff * center_pos_x_diff;    //!< (x_i - x_j)^2
                double center_pos_y_diff_square = center_pos_y_diff * center_pos_y_diff;    //!< (y_i - y_j)^2

                // square of the sum of the radii
                double radius_sum_square = radius_sum * radius_sum; //!< (r_i + r_j)^2

                // prepare coefficients of the polynomial P(t) = b_0 + b_1 * t^1 + b_2 * t^2 + b_3 * t^3 + b_4 * t^4
                // stored upwards from least power: [b_0, b_1, b_2, b_3, b_4]

                CircleEquationSolver CES;
                std::array<double, POLYNOMIAL_ARRAY_LENGTH> coefficients;
                coefficients[0] = CES.computeCoefficientForPowerZero(center_pos_x_diff_square, center_pos_y_diff_square, radius_sum_square);
                coefficients[1] = CES.computeCoefficientForPowerOne(speed_diff_sin_adjusted, speed_diff_cos_adjusted, center_pos_x_diff, center_pos_y_diff);
                coefficients[2] = CES.computeCoefficientForPowerTwo(accel_diff_sin_adjusted, accel_diff_cos_adjusted, speed_diff_square_sin_adjusted, speed_diff_square_cos_adjusted, center_pos_x_diff, center_pos_y_diff);
                coefficients[3] = CES.computeCoefficientForPowerThree(accel_diff_sin_adjusted, accel_diff_cos_adjusted, speed_diff_sin_adjusted, speed_diff_cos_adjusted);
                coefficients[4] = CES.computeCoefficientForPowerFour(accel_diff_square_sin_adjusted, accel_diff_square_cos_adjusted);

                // compute the real roots of the polynomial equation with GSL
                std::vector<double> real_positive_roots = CES.solvePolynomialEquationGSL(coefficients);

                if (real_positive_roots.empty())
                {
                    // no real positive root found --> no TTC could be computed
                    RCLCPP_DEBUG(node_->get_logger(),"CircleAlgorithm::calculateTTC: no real positive roots found.");

                    // return empty optional
                    return ttc_optional;
                }

                // smallest real positive root is the TTC
                ttc_optional = *std::min_element(real_positive_roots.begin(), real_positive_roots.end());

                return ttc_optional;


            //void initialize(double side_length) override 
            //{
            //    side_length_ = side_length;
            //}
            

            //double area() override 
            //{
            //    return side_length_*side_length_;
            //}

            //double computeCoefficientForPowerFour(double &accel_diff_sq_sin_adj, double &accel_diff_sq_cos_adj)
            //{
            //    return (accel_diff_sq_sin_adj + accel_diff_sq_cos_adj) / 4;
            //}

            //protected:
            //    double side_length_;
                
            //std::optional<double> calculateTTC(
            //const object_motion_t &subject_object_motion,
            //const object_motion_t &perceived_object_motion
            //) override
            //{
            //    return 
            //}
            };
    };

    class NCircleAlgorithm : public ros2_collision_detection::TTCAlgorithm, public NodeInitializer
    {
        
        public:
            NCircleAlgorithm()
            
            {
                RCLCPP_INFO(node_->get_logger(), "CircleAlgorithm constructed");
                //this->n = 10;
            }
            
            ros2_collision_detection::NCircleAlgorithm NCA;

            //int n_k = NCA.get_n(){return n;}
            int n_k = NCA.get_n();
            //cout  << n_k;

            void initialize(ros2_collision_detection::parameter_map_t &parameter_map) override
            {
                //ros2_collision_detection::NCircleAlgorithm NCA;
                
                try
                {
                    std::variant<int, std::string> variant_circle_count = parameter_map.at("ttc_algorithm_circle_count");
                    if(int *circle_count_ptr = std::get_if<int>(&variant_circle_count))
                    {
                        int circle_count = *circle_count_ptr;
                        if(circle_count >= 1)
                        {
                            //ros2_collision_detection::NCircleAlgorithm NCA;
                            //n_k= NCA.set_n(circle_count);
                            n_k = circle_count;
                            RCLCPP_INFO(node_->get_logger(),"NCircleAlgorithm::init with n = %d.", circle_count);
                        }
                    }
                    else
                    {
                        //n_k=NCA.set_n(DEFAULT_CIRCLE_COUNT);
                        n_k = DEFAULT_CIRCLE_COUNT;
                        RCLCPP_ERROR(node_->get_logger(),"NCircleAlgorithm::init: 'ttc_algorithm_circle_count' could not be retrieved. Using default n=%d.", DEFAULT_CIRCLE_COUNT);
                    }
                }
                catch(const std::out_of_range &e)
                {
                    //n_k=CA.set_n(DEFAULT_CIRCLE_COUNT);
                    n_k = DEFAULT_CIRCLE_COUNT;
                    RCLCPP_ERROR(node_->get_logger(),"NCircleAlgorithm::init: no 'ttc_algorithm_circle_count' found. Using default n=%d.", DEFAULT_CIRCLE_COUNT);
                }
                //RCLCPP_INFO(rclcpp::get_logger("NCircleAlgorithm"), "NCircleAlgorithm does  initialize.");
            }

            std::string convertMotionStructToString(const ros2_collision_detection::object_motion_t& object_motion)
            {
            	std::stringstream result;

                result << "center_pos_x = " << object_motion.center_pos_x << std::endl;
            	result << "center_pos_y = " << object_motion.center_pos_y << std::endl;
            	result << "length = " << object_motion.length << std::endl;
            	result << "width = " << object_motion.width << std::endl;
            	result << "heading = " << object_motion.heading << std::endl;
            	result << "speed = " << object_motion.speed << std::endl;
            	result << "acceleration = " << object_motion.acceleration << std::endl;

                return result.str();
            }

            double computeSinFromHeading(const float &heading)
            {
                double result = sin(heading * M_PI / 180.0);
                return result;
            }

            double computeCosFromHeading(const float &heading)
            {
                double result = cos(heading * M_PI / 180.0);
                return result;
            }

            double computeHeadingAdjustedValue(const float &value_to_adjust, const double &trigonometric_value)
            {
                return trigonometric_value * value_to_adjust;
            }
            
            double computeFrontBumperPos(const float &center_pos, const double &trigonometric_value, const float &length)
            {
                double result = center_pos + (trigonometric_value * length / 2);
                RCLCPP_DEBUG(node_->get_logger(),"NCircleAlgorithm::computeFrontBumperPos: result: %f | from value: %f, trigonometry: %f, length: %f.", result, center_pos, trigonometric_value, length);
                return result;
            }

            std::vector<std::array<double, 2>> computeAllCircleCenters(const double &front_bumper_pos_x, const double &front_bumper_pos_y, const double &sin_heading, const double &cos_heading, const double &length, const int &circle_count)
            {
                std::vector<std::array<double, 2>> circles;

                for(int i = 0; i < circle_count; i++)
                {
                    int factor = i + 1;
                    double part_length = length / (circle_count + 1);
                    std::array<double, 2> circle_i_pos;
                    circle_i_pos[0] = computeCircleCenter(front_bumper_pos_x, factor, sin_heading, part_length);
                    circle_i_pos[1] = computeCircleCenter(front_bumper_pos_y, factor, cos_heading, part_length);
                    circles.push_back(circle_i_pos);
                }

                RCLCPP_DEBUG(node_->get_logger(),"computeAllCircleCenters: all circles from: (%f,%f) with sin: %f, cos: %f, length: %f, circle_count: %d.", front_bumper_pos_x, front_bumper_pos_y, sin_heading, cos_heading, length, circle_count);
                for(std::vector<std::array<double, 2>>::iterator it = circles.begin(); it != circles.end(); ++it)
                {
                    RCLCPP_DEBUG(node_->get_logger(),"circle center: (%f,%f).", (*it)[0], (*it)[1]);
                }
                return circles;
            }

            double computeCircleCenter(const double &front_bumper_pos, const int &factor, const double &trigonometric_value, const double &part_length)
            {
                return front_bumper_pos - factor * (trigonometric_value * part_length);
            }


            double computeRadius(const float &length, const float &width, const int &circle_count)
            {
                // radius = sqrt( (length / (n+1))^2 + (width / 2)^2 )
                
                double part_length = length / (n_k + 1);
                double half_width = width / 2;
                double result = sqrt(part_length * part_length + half_width * half_width);
                RCLCPP_DEBUG(node_->get_logger(),"NCircleAlgorithm::computeRadius: radius = %f | length: %f, width: %f, circle_count: %d", result, length, width, circle_count);
                return result;
            }

            std::vector<double> calculatePossibleTTCs(const ros2_collision_detection::object_motion_t &subject_object_motion, const ros2_collision_detection::object_motion_t &perceived_object_motion, int circle_count)
            {
                // list of all computed TTCs from the collision checks between all subject object's circles and perceived object's circles
                std::vector<double> possible_ttc_list;

                // common values necessary for all circles
                double sin_subject_obj_heading = computeSinFromHeading(subject_object_motion.heading);      //!< sin(alpha)
                double cos_subject_obj_heading = computeCosFromHeading(subject_object_motion.heading);      //!< cos(alpha)
                double sin_perceived_obj_heading = computeSinFromHeading(perceived_object_motion.heading);  //!< sin(beta)
                double cos_perceived_obj_heading = computeCosFromHeading(perceived_object_motion.heading);  //!< cos(beta)

                double accel_subject_obj_sin_adjusted = computeHeadingAdjustedValue(subject_object_motion.acceleration, sin_subject_obj_heading);       //!< sin(alpha) * a_i
                double accel_subject_obj_cos_adjusted = computeHeadingAdjustedValue(subject_object_motion.acceleration, cos_subject_obj_heading);       //!< cos(alpha) * a_i
                double accel_perceived_obj_sin_adjusted = computeHeadingAdjustedValue(perceived_object_motion.acceleration, sin_perceived_obj_heading); //!< sin(beta) * a_j
                double accel_perceived_obj_cos_adjusted = computeHeadingAdjustedValue(perceived_object_motion.acceleration, cos_perceived_obj_heading); //!< cos(beta) * a_j

                double speed_subject_obj_sin_adjusted = computeHeadingAdjustedValue(subject_object_motion.speed, sin_subject_obj_heading);          //!< sin(alpha) * v_i
                double speed_subject_obj_cos_adjusted = computeHeadingAdjustedValue(subject_object_motion.speed, cos_subject_obj_heading);          //!< cos(alpha) * v_i
                double speed_perceived_obj_sin_adjusted = computeHeadingAdjustedValue(perceived_object_motion.speed, sin_perceived_obj_heading);    //!< sin(beta) * v_j
                double speed_perceived_obj_cos_adjusted = computeHeadingAdjustedValue(perceived_object_motion.speed, cos_perceived_obj_heading);    //!< cos(beta) * v_j

                // differences between subject object's values and perceived object's values
                double accel_diff_sin_adjusted = accel_subject_obj_sin_adjusted - accel_perceived_obj_sin_adjusted; //!< sin(alpha) * a_i - sin(beta) * a_j
                double accel_diff_cos_adjusted = accel_subject_obj_cos_adjusted - accel_perceived_obj_cos_adjusted; //!< cos(alpha) * a_i - cos(beta) * a_j
                double speed_diff_sin_adjusted = speed_subject_obj_sin_adjusted - speed_perceived_obj_sin_adjusted; //!< sin(alpha) * v_i - sin(beta) * v_j
                double speed_diff_cos_adjusted = speed_subject_obj_cos_adjusted - speed_perceived_obj_cos_adjusted; //!< cos(alpha) * v_i - cos(beta) * v_j

                // squares of the differences
                double accel_diff_square_sin_adjusted = accel_diff_sin_adjusted * accel_diff_sin_adjusted;  //!< (sin(alpha) * a_i - sin(beta) * a_j)^2
                double accel_diff_square_cos_adjusted = accel_diff_cos_adjusted * accel_diff_cos_adjusted;  //!< (cos(alpha) * a_i - cos(beta) * a_j)^2
                double speed_diff_square_sin_adjusted = speed_diff_sin_adjusted * speed_diff_sin_adjusted;  //!< (sin(alpha) * v_i - sin(beta) * v_j)^2
                double speed_diff_square_cos_adjusted = speed_diff_cos_adjusted * speed_diff_cos_adjusted;  //!< (cos(alpha) * v_i - cos(beta) * v_j)^2

                // compute front bumper position for subject object and perceived object
                double front_bumper_pos_x_subject_obj = computeFrontBumperPos(subject_object_motion.center_pos_x, sin_subject_obj_heading, subject_object_motion.length);
                double front_bumper_pos_y_subject_obj = computeFrontBumperPos(subject_object_motion.center_pos_y, cos_subject_obj_heading, subject_object_motion.length);

                double front_bumper_pos_x_perceived_obj = computeFrontBumperPos(perceived_object_motion.center_pos_x, sin_perceived_obj_heading, perceived_object_motion.length);
                double front_bumper_pos_y_perceived_obj = computeFrontBumperPos(perceived_object_motion.center_pos_y, cos_perceived_obj_heading, perceived_object_motion.length);

                // split the line [front_bumper_pos, rear_bumper_pos] for subject object and perceived object into n+1 equally long parts and
                // compute coordinates of the n circle centers for subject object and perceived object
                std::vector<std::array<double, 2>> circles_subject_obj = computeAllCircleCenters(front_bumper_pos_x_subject_obj, front_bumper_pos_y_subject_obj, sin_subject_obj_heading, cos_subject_obj_heading, subject_object_motion.length, n_k);
                std::vector<std::array<double, 2>> circles_perceived_obj = computeAllCircleCenters(front_bumper_pos_x_perceived_obj, front_bumper_pos_y_perceived_obj, sin_perceived_obj_heading, cos_perceived_obj_heading, perceived_object_motion.length, n_k);

                // compute the radius for the n circles for subject object and perceived object
                double circle_radius_subject_obj = computeRadius(subject_object_motion.length, subject_object_motion.width, n_k);        //!< r_i
                double circle_radius_perceived_obj = computeRadius(perceived_object_motion.length, perceived_object_motion.width, n_k);  //!< r_j

                // sum of the radii of subject object and perceived object
                double radius_sum = circle_radius_subject_obj + circle_radius_perceived_obj;  //!< r_i + r_j

                // square of the sum of the radii
                double radius_sum_square = radius_sum * radius_sum; //!< (r_i + r_j)^2

                // the coefficients for polynomial equation stored upwards from least power: [b_0, b_1, b_2, b_3, b_4]
                std::array<double, POLYNOMIAL_ARRAY_LENGTH> coefficients;

                // the real roots of the polynomial equation
                std::vector<double> real_positive_roots;

                // collision check between all subject obj circles and perceived obj circles
                for(int i = 0; i < n_k; i++)      // for all subject obj circles
                {
                    for(int j = 0; j < n_k; j++)  // for all perceived obj circles
                    {
                        RCLCPP_DEBUG(node_->get_logger(),"NCircleAlgorithm::calculatePossibleTTCs Computing for i=%d, j=%d: subject (%f,%f) | perceived (%f,%f).", i, j, circles_subject_obj.at(i).at(0), circles_subject_obj.at(i).at(1), circles_perceived_obj.at(j).at(0), circles_perceived_obj.at(j).at(1));
                        double circle_center_pos_x_diff = circles_subject_obj.at(i).at(0) - circles_perceived_obj.at(j).at(0);   //!< x_i - x_j
                        double circle_center_pos_y_diff = circles_subject_obj.at(i).at(1) - circles_perceived_obj.at(j).at(1);   //!< y_i - y_j

                        // squares of the differences        
                        double circle_center_pos_x_diff_square = circle_center_pos_x_diff * circle_center_pos_x_diff;    //!< (x_i - x_j)^2
                        double circle_center_pos_y_diff_square = circle_center_pos_y_diff * circle_center_pos_y_diff;    //!< (y_i - y_j)^2

                        CircleEquationSolver CES;

                        // prepare coefficients of the polynomial P(t) = b_0 + b_1 * t^1 + b_2 * t^2 + b_3 * t^3 + b_4 * t^4
                        coefficients[0] = CES.computeCoefficientForPowerZero(circle_center_pos_x_diff_square, circle_center_pos_y_diff_square, radius_sum_square);
                        coefficients[1] = CES.computeCoefficientForPowerOne(speed_diff_sin_adjusted, speed_diff_cos_adjusted, circle_center_pos_x_diff, circle_center_pos_y_diff);
                        coefficients[2] = CES.computeCoefficientForPowerTwo(accel_diff_sin_adjusted, accel_diff_cos_adjusted, speed_diff_square_sin_adjusted, speed_diff_square_cos_adjusted, circle_center_pos_x_diff, circle_center_pos_y_diff);
                        coefficients[3] = CES.computeCoefficientForPowerThree(accel_diff_sin_adjusted, accel_diff_cos_adjusted, speed_diff_sin_adjusted, speed_diff_cos_adjusted);
                        coefficients[4] = CES.computeCoefficientForPowerFour(accel_diff_square_sin_adjusted, accel_diff_square_cos_adjusted);

                        // compute the real roots of the polynomial equation with GSL
                        real_positive_roots = CES.solvePolynomialEquationGSL(coefficients);

                        if (real_positive_roots.empty())
                        {
                            // no real positive root found --> no TTC could be computed
                            // no TTC between the two currently used circles
                            RCLCPP_DEBUG(node_->get_logger(),"NCircleAlgorithm::calculatePossibleTTCs: no real positive roots found.");
                        }
                        else
                        {
                            // at least one real positive root found
                            // smallest real positive root is the TTC for the two currently used circles
                            double ttc_circles_i_j = *std::min_element(real_positive_roots.begin(), real_positive_roots.end());
                            possible_ttc_list.push_back(ttc_circles_i_j);
                        }

                    }
                }

                return possible_ttc_list;
            }

            std::optional<double> calculateTTC(const ros2_collision_detection::object_motion_t &subject_object_motion, const ros2_collision_detection::object_motion_t &perceived_object_motion)
            {   
                // the Time-To-Collision optional return value
                std::optional<double> ttc_optional;

                // log the received object motions
                RCLCPP_DEBUG_STREAM(node_->get_logger(),"\nsubject object motion: \n" << convertMotionStructToString(subject_object_motion));
                RCLCPP_DEBUG_STREAM(node_->get_logger(),"perceived object motion: \n" << convertMotionStructToString(perceived_object_motion));

                if(subject_object_motion.length <= 0 || subject_object_motion.width <= 0 || perceived_object_motion.length <= 0 || perceived_object_motion.width <= 0)
                {
                    RCLCPP_ERROR(node_->get_logger(),"NCircleAlgorithm::calculateTTC: length or width is not allowed to be zero or lower.");
                    return ttc_optional;
                }

                if(subject_object_motion.heading < 0 || subject_object_motion.heading > 360 || perceived_object_motion.heading < 0 || perceived_object_motion.heading > 360)
                {
                    RCLCPP_ERROR(node_->get_logger(),"NCircleAlgorithm::calculateTTC: heading value must be in range [0; 360].");
                    return ttc_optional;
                }

                // calculate all possible TTCs between subject_object_motion and perceived_object_motion, each represented by n circles
                std::vector<double> possible_ttc_list = calculatePossibleTTCs(subject_object_motion, perceived_object_motion, n_k);

                if(possible_ttc_list.empty())
                {
                    // no possible TTC computed between any of the circles --> no TTC could be computed
                    RCLCPP_INFO(node_->get_logger(),"NCircleAlgorithm::calculateTTC: no TTC could be found.");

                    // return empty optional
                    return ttc_optional;
                }

                // smallest of the possible TTCs is the TTC
                RCLCPP_ERROR(node_->get_logger(),"NCircleAlgorithm::calculateTTC: list of possible TTCs:");
                for(int i = 0; i < possible_ttc_list.size(); i++)
                {RCLCPP_ERROR(node_->get_logger(),"possible ttc [%d] = %f", i, possible_ttc_list[i]);
                }

                ttc_optional = *std::min_element(possible_ttc_list.begin(), possible_ttc_list.end());

                return ttc_optional;
            }

            //@todo: below part needs to be modified

            
            
        
    };

    

} // namespace ros2_collision_detection_plugins


#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(ros2_collision_detection_plugins::CircleAlgorithm, ros2_collision_detection::TTCAlgorithm)
PLUGINLIB_EXPORT_CLASS(ros2_collision_detection_plugins::NCircleAlgorithm, ros2_collision_detection::TTCAlgorithm)



