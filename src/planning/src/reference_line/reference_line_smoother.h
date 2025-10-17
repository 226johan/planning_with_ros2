#ifndef REFERENCE_LINE_SMOOTHER_H_
#define REFERENCE_LINE_SMOOTHER_H_

#include "rclcpp/rclcpp.hpp"
#include "base_msgs/msg/referline.hpp"
#include "config_reader.h"
#include <Eigen/Dense>
#include <OsqpEigen/OsqpEigen.h>
#include <cmath>

namespace Planning
{
    using base_msgs::msg::Referline;

    class ReferenceLineSmoother // 参考线平滑
    {
    public:
        ReferenceLineSmoother();

        void smooth_reference_line(Referline &refer_line);
        bool OsqpSolver(const int &n,
                        const Eigen::SparseMatrix<double> &P,
                        Eigen::VectorXd Q,
                        const Eigen::SparseMatrix<double> &A,
                        Eigen::VectorXd lowerBound,
                        Eigen::VectorXd upperBound,
                        Eigen::VectorXd &QPsolution);

    private:
        std::unique_ptr<ConfigReader> reference_line_config_;
        const double w1 = 100.0;
        const double w2 = 10.0;
        const double w3 = 1.0;
    };
} // namespace Planning
#endif // REFERENCE_LINE_SMOOTHER_H_
