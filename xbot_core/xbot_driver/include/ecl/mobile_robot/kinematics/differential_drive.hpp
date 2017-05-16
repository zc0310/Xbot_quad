/**
 * @file /ecl_mobile_robot/include/ecl/mobile_robot/kinematics/differential_drive.hpp
 *
 * @brief Kinematics equations for differential drive type bases.
 *
 * @date 20/05/2010
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_MOBILE_ROBOT_DIFFERENTIAL_DRIVE_KINEMATICS_HPP_
#define ECL_MOBILE_ROBOT_DIFFERENTIAL_DRIVE_KINEMATICS_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ecl/config/macros.hpp>
#include <ecl/linear_algebra.hpp>
#include <ecl/geometry/pose2d.hpp>
#include "../macros.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {
namespace mobile_robot {

/*****************************************************************************
** Interface
*****************************************************************************/

/**
 * @brief Differential drive kinematics.
 *
 * This presents some of the differential drive kinematics transforms
 * for a standard differential drive mobile base. The general structure is
 * for a  fixed axis between two separately controlled wheels, usually
 * you can find a balancing castor wheel(平衡脚轮) of some sort to the front or rear.
 *
 * @code
 *
 *        o
 *        |
 *        |
 *   ||-------||
 *
 * @endcode
 *
 * There are three sets of systems used to describe the
 * kinematics.
 *
 * - /dot{theta}_L, /dot{theta}_R  : wheel angular velocities.                轮子角速度
 * - /dot{s}, /omega : base linear speed and angular rotational velocities.   基线速度和角旋转速度
 * - /dot{x}, /dot{y}, /dot{phi} : pose velocities wrt global frame.
 *
 * For all of these conversions, it is assumed the forward direction is the
 * x axis for the local pose frame of reference and the frame of reference
 * is located centrally between the two wheels.对于所有这些转换，假设向前方向是局部姿势参考系的x轴，并且参考系位于两个轮之间的中心
 */
class ecl_mobile_robot_PUBLIC DifferentialDriveKinematics {
public:
	/**
	 * @brief Configures the parameters defining the structure.
	 *
	 * @param fixed_axis_length : length between the two wheels [m].
	 * @param wheel_radius : radius of each differential drive wheel [m].
	 */
  DifferentialDriveKinematics( //传入两参数：轴距和轮半径
			const double &fixed_axis_length,
      const double &wheel_radius) :
		bias(fixed_axis_length),
		radius(wheel_radius)
	{}

	/**
   * @brief Generates a relative (to the robot's frame) pose differential.生成相对（机器人的框架）位置差动(微分)
	 *
   * Uses the odometry updates to generate a relative pose update. //使用里程计(编码器)进行位置更新
	 * This differential can be used to update (add to) the robot's current
	 * pose estimate.
	 *
	 * @code
	 * Pose2D pose;
	 * // ...
	 * pose *= forwardDifferential(0.1,0.2);
	 * @endcode
	 *
   * @param dleft : incoming left wheel angle change.   输入左轮角度变化量
   * @param dright : incoming right wheel angle change. 输入右轮角度变化量
	 * @return Pose : pose update (differential).
	 */
  ecl::Pose2D<double> forward(const double &dleft, const double &dright) const; //返回二维位置

	/**
	 * @brief Generates a relative (to the robot's frame) pose differential
	 * when you know platform velocity directly. In this case you do not need to
   * know radius informaiton   当知道平台速度时，生成位置差分，此时不需要知道轮子半径
	 *
	 * @code
	 * Pose2D pose;
	 * // ...
	 * double linearVelocity = ....;
	 * double angularVelocity = ..../
	 * pose *= forwardDifferential(linearVelocity, angularVelocity);
	 *
   * @param linearVelocity : incoming linear velocity of platform    输入平台线速度
   * @param angularVelocity : incoming angular velocity of platform  输入平台角速度
	 * @return Pose : pose update (differential)
	 */
	ecl::Pose2D<double> forwardWithPlatformVelocity(const double & linearVelocity, const double & angularVelocity ) const;

	/**
   * @brief Generates wheel angle rates from scalar linear/angular velocity commands.从标量线速度/角速度命令转换生成车轮角速率
	 *
   * Accepts the usual differential drive separation of linear and angular       接受线速度/角速度指令生成用于控制的轮子角速率
	 * velocity components and computes the required wheel angle rates for control.
	 *
	 * @param linear_velocity : linear translation control command (in direction of facing).
	 * @param angular_velocity : angular rotation control command (around robot centre).
   * @return Vector2d : left and right wheel angular rate commands respectively. 返回左右轮各自的角速率
	 */
	ecl::linear_algebra::Vector2d inverse(const double &linear_velocity, const double &angular_velocity) const;

	/**
   * @brief Rough conversion from [dx,dy,dtheta] -> [ds, dw]. 降维  计算两姿态之间的变换
	 *
	 * The aim here is to provide a differential between the two poses that is
	 * crudely approximated for non-holonomic motion (ds - linear translation, dw -
	 * angular rotation).
	 *
	 * Moving in this direction provides a non-unique solution (reducing from
	 * 3 dof to 2 dof). If the update period is very small, this limit does,
	 * however, approach a unique solution. Consequently, so long as this
	 * is small, the following is a reasonably simple, lightweight method.
	 *
	 * The ds is simply the scalar distance between two poses, given a sign
	 * for the direction it is pointing in (relative to the robot facing). Alternatively
	 * you could assume an angle + linear translation + angle or some such, but
	 * even then, its still an approximation.
	 *
	 * The angular difference, dw is more simple, it is always just the
	 * angular difference between poses.
	 *
	 * Jae-Yeong was talking about moving this to firmware for better accuracy
	 * (smaller update periods).
	 *
	 * @param a : initial pose
	 * @param b : final pose
	 * @return Vector2d : the differential [ds, dw].
	 */
	ECL_DEPRECATED static ecl::linear_algebra::Vector2d Inverse(const ecl::linear_algebra::Vector3d &a, const ecl::linear_algebra::Vector3d &b);

	/**
   * @brief Rough conversion from a pose difference -> [ds, dw].  计算两位置之间的变换
	 *
	 * The aim here is to provide a differential between the two poses that is
	 * crudely approximated for non-holonomic motion (ds - linear translation, dw -
	 * angular rotation).
	 *
	 * Moving in this direction provides a non-unique solution (reducing from
	 * 3 dof to 2 dof). If the update period is very small, this limit does,
	 * however, approach a unique solution. Consequently, so long as this
	 * is small, the following is a reasonably simple, lightweight method.
	 *
	 * The ds is simply the scalar distance between two poses, given a sign
	 * for the direction it is pointing in (relative to the robot facing). Alternatively
	 * you could assume an angle + linear translation + angle or some such, but
	 * even then, its still an approximation.
	 *
	 * The angular difference, dw is more simple, it is always just the
	 * angular difference between poses.
	 *
	 * Jae-Yeong was talking about moving this to firmware for better accuracy
	 * (smaller update periods).
	 *
	 * @param a : initial pose
	 * @param b : final pose
	 * @return Vector2d : the differential [ds, dw].
	 */
	static ecl::linear_algebra::Vector2d PartialInverse(const ecl::Pose2D<double> &a, const ecl::Pose2D<double> &b);
private:
	double bias, radius;
};

} // namespace mobile_robot
} // namespace ecl

#endif /* ECL_MOBILE_ROBOT_DIFFERENTIAL_DRIVE_KINEMATICS_HPP_ */
