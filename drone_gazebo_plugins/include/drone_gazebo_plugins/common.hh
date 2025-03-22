#ifndef SITL_GAZEBO_COMMON_HH_
#define SITL_GAZEBO_COMMON_HH_

#include <Eigen/Dense>
#include <cmath>
#include <gz/common/Console.hh>
#include <gz/math.hh>
#include <sdf/sdf.hh>
#include <string>
#include <tinyxml.h>
#include <typeinfo>
#include <utility>

namespace gazebo {

// Returns scalar value constrained by (min_val, max_val)
template <typename Scalar>
static inline constexpr const Scalar &
constrain (const Scalar &val, const Scalar &min_val, const Scalar &max_val)
{
    return (val < min_val) ? min_val : ((val > max_val) ? max_val : val);
}

/**
 * \brief Obtains a parameter from sdf.
 * \param[in] sdf Pointer to the sdf object.
 * \param[in] name Name of the parameter.
 * \param[out] param Param Variable to write the parameter to.
 * \param[in] default_value Default value, if the parameter not available.
 * \param[in] verbose If true, ignerror if the parameter is not available.
 */
template <class T>
bool getSdfParam (const std::shared_ptr<const sdf::Element> sdf,
                  const std::string &name, T &param, const T &default_value,
                  const bool &verbose = false)
{
    if (sdf->HasElement (name)) {
        param = sdf->Get<T> (name);
        return true;
    }
    else {
        param = default_value;
        if (verbose)
            gzerr << "[drone_gazebo_plugins] Please specify a value for "
                     "parameter \""
                  << name << "\".\n";
    }
    return false;
}

/**
 * \brief Get a gz::math::Angle as an angle from [0, 360)
 */
inline double GetDegrees360 (const gz::math::Angle &angle)
{
    double degrees = angle.Degree ();
    while (degrees < 0.)
        degrees += 360.0;
    while (degrees >= 360.0)
        degrees -= 360.0;
    return degrees;
}

} // namespace gazebo

template <typename T> class FirstOrderFilter {
    /*
    This class can be used to apply a first order filter on a signal.
    It allows different acceleration and deceleration time constants.

    Short reveiw of discrete time implementation of firest order system:
    Laplace:
        X(s)/U(s) = 1/(tau*s + 1)
    continous time system:
        dx(t) = (-1/tau)*x(t) + (1/tau)*u(t)
    discretized system (ZoH):
        x(k+1) = exp(samplingTime*(-1/tau))*x(k) + (1 -
exp(samplingTime*(-1/tau)))
* u(k)
*/
  public:
    FirstOrderFilter (double timeConstantUp, double timeConstantDown,
                      T initialState)
        : timeConstantUp_ (timeConstantUp),
          timeConstantDown_ (timeConstantDown), previousState_ (initialState)
    {
    }
    T updateFilter (T inputState, double samplingTime)
    {
        T outputState;
        if (inputState > previousState_) {
            double alphaUp = exp (-samplingTime / timeConstantUp_);
            outputState = alphaUp * previousState_ + (1 - alphaUp) * inputState;
        }
        else {
            double alphaDown = exp (-samplingTime / timeConstantDown_);
            outputState =
                alphaDown * previousState_ + (1 - alphaDown) * inputState;
        }
        previousState_ = outputState;
        return outputState;
    }
    ~FirstOrderFilter ()
    {
    }

  protected:
    double timeConstantUp_;
    double timeConstantDown_;
    T previousState_;
};

template <class Derived>
Eigen::Quaternion<typename Derived::Scalar>
QuaternionFromSmallAngle (const Eigen::MatrixBase<Derived> &theta)
{
    typedef typename Derived::Scalar Scalar;
    EIGEN_STATIC_ASSERT_FIXED_SIZE (Derived);
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE (Derived, 3);
    const Scalar q_squared = theta.squaredNorm () / 4.0;
    if (q_squared < 1) {
        return Eigen::Quaternion<Scalar> (sqrt (1 - q_squared), theta[0] * 0.5,
                                          theta[1] * 0.5, theta[2] * 0.5);
    }
    else {
        const Scalar w = 1.0 / sqrt (1 + q_squared);
        const Scalar f = w * 0.5;
        return Eigen::Quaternion<Scalar> (w, theta[0] * f, theta[1] * f,
                                          theta[2] * f);
    }
}

template <class In, class Out> void copyPosition (const In &in, Out *out)
{
    out->x = in.x;
    out->y = in.y;
    out->z = in.z;
}

// Frames of reference:
// g - gazebo (ENU), east, north, up
// r - rotors imu frame (FLU), forward, left, up
// b - px4 (FRD) forward, right, down
// n - px4 (NED) north, east, down

/**
 * @brief Quaternion for rotation between ENU and NED frames
 *
 * NED to ENU: +PI/2 rotation about Z (Down) followed by a +PI rotation around X
 * (old North/new East) ENU to NED: +PI/2 rotation about Z (Up) followed by a
 * +PI rotation about X (old East/new North) This rotation is symmetric, so
 * q_ENU_to_NED == q_NED_to_ENU.
 */

static const auto q_ENU_to_NED = gz::math::Quaterniond (0, 0.70711, 0.70711, 0);

/**
 * @brief Quaternion for rotation between body FLU and body FRD frames
 *
 * +PI rotation around X (Forward) axis rotates from Forward, Right, Down
 * (aircraft) to Forward, Left, Up (base_link) frames and vice-versa. This
 * rotation is symmetric, so q_FLU_to_FRD == q_FRD_to_FLU.
 */
static const auto q_FLU_to_FRD = gz::math::Quaterniond (0, 1, 0, 0);

// sensor X-axis unit vector in `base_link` frame
static const gz::math::Vector3d kDownwardRotation =
    gz::math::Vector3d (0, 0, -1);
static const gz::math::Vector3d kUpwardRotation = gz::math::Vector3d (0, 0, 1);
static const gz::math::Vector3d kBackwardRotation =
    gz::math::Vector3d (-1, 0, 0);
static const gz::math::Vector3d kForwardRotation = gz::math::Vector3d (1, 0, 0);
static const gz::math::Vector3d kLeftRotation = gz::math::Vector3d (0, 1, 0);
static const gz::math::Vector3d kRightRotation = gz::math::Vector3d (0, -1, 0);

// Zurich Irchel Park defaults:
static constexpr const double kDefaultHomeLatitude =
    47.397742 * M_PI / 180.0; // rad
static constexpr const double kDefaultHomeLongitude =
    8.545594 * M_PI / 180.0;                                // rad
static constexpr const double kDefaultHomeAltitude = 488.0; // Meters

// Earth radius in meters
static constexpr const double earth_radius = 6353000.0;

// Defaults for GPS noise parameters
static constexpr double kDefaultGpsXYRandomWalk = 2.0;
static constexpr double kDefaultGpsZRandomWalk = 4.0;
static constexpr double kDefaultGpsXYNoiseDensity = 0.0002;
static constexpr double kDefaultGpsZNoiseDensity = 0.0004;
static constexpr double kDefaultGpsVXYNoiseDensity = 0.2;
static constexpr double kDefaultGpsVZNoiseDensity = 0.4;
static constexpr double kDefaultUpdateRate = 5.0;

/**
 * @brief Get latitude and longitude coordinates from local position
 * @param[in] pos position in the local frame
 * @return std::pair of Latitude and Longitude
 */
inline std::pair<double, double> reproject (gz::math::Vector3d &pos,
                                            double &lat_home, double &lon_home,
                                            double &alt_home)
{

    // reproject local position to gps coordinates
    const double x_rad = pos.Y () / earth_radius; // north
    const double y_rad = pos.X () / earth_radius; // east
    const double c = sqrt (x_rad * x_rad + y_rad * y_rad);
    const double sin_c = sin (c);
    const double cos_c = cos (c);
    double lat_rad, lon_rad;
    if (c != 0.0) {
        lat_rad = asin (cos_c * sin (lat_home) +
                        (x_rad * sin_c * cos (lat_home)) / c);
        lon_rad = lon_home +
                  atan2 (y_rad * sin_c, c * cos (lat_home) * cos_c -
                                            x_rad * sin (lat_home) * sin_c);
    }
    else {
        lat_rad = lat_home;
        lon_rad = lon_home;
    }
    return std::make_pair (lat_rad, lon_rad);
}

#endif // SITL_GAZEBO_COMMON_HH_
