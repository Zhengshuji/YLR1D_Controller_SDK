#ifndef REIGEN_HPP
#define REIGEN_HPP

#include <iostream>
#include <Eigen/Dense>

namespace robsoft{

typedef Eigen::Vector3d Vector3d;
typedef Eigen::Vector4d Vector4d;
typedef Eigen::VectorXd VectorXd;
typedef Eigen::Matrix<double, 5, 1> Vector5d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix3d Matrix3d;
typedef Eigen::Matrix4d Matrix4d;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::MatrixXd MatrixXd;
typedef Eigen::Matrix<double, 1, 3> Matrix13d;
typedef Eigen::Matrix<double, 2, 4> Matrix24d;
typedef Eigen::MatrixX3d MatrixX3d;
typedef Eigen::Matrix3Xd Matrix3Xd;
typedef Eigen::Matrix<double, 6, Eigen::Dynamic> Matrix6Xd;
typedef Eigen::Matrix<double, Eigen::Dynamic, 6> MatrixX6d;

Matrix6d matrix_diagonal(const Vector6d& vec);
MatrixX6d matrix_pseudo_inverse(const Matrix6Xd& mat);
bool matrix_is_invertible(const Matrix6d& mat);
bool matrix_is_invertible(const MatrixXd& mat);
void matrix_remove_row(MatrixX6d& mat, int rowIndex);
void matrix_remove_col(Matrix6Xd& mat, int colIndex);
VectorXd matrix_solve_linear_equations(const MatrixXd& coe, const VectorXd& b);

}
#endif
