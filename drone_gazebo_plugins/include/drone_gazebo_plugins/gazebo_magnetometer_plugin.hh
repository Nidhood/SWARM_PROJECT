#ifndef MAGNETOMETER_PLUGIN_HH_
#define MAGNETOMETER_PLUGIN_HH_

#include <random>
#include <string>

#include <Eigen/Core>
#include <boost/shared_array.hpp>

#include <Groundtruth.pb.h>
#include <MagneticField.pb.h>

#include <gz/math.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/transport/Node.hh>

#include "geo_mag_declination.hh"
#include <common.hh>

namespace magnetometer_plugin {

static constexpr auto kDefaultMagnetometerTopic = "mag";
static constexpr auto kDefaultPubRate =
    100.0; // [Hz]. Corresponde a la mayoría de los dispositivos magnéticos
           // soportados en PX4

// Valores por defecto para el uso con el IMU ADIS16448
static constexpr auto kDefaultNoiseDensity = 0.4 * 1e-3; // [gauss / sqrt(hz)]
static constexpr auto kDefaultRandomWalk = 6.4 * 1e-6;   // [gauss * sqrt(hz)]
static constexpr auto kDefaultBiasCorrelationTime = 6.0e+2; // [s]

typedef const boost::shared_ptr<const sensor_msgs::msgs::Groundtruth> GtPtr;

class GZ_SIM_VISIBLE MagnetometerPlugin : public gz::sim::System,
                                          public gz::sim::ISystemConfigure,
                                          public gz::sim::ISystemPreUpdate,
                                          public gz::sim::ISystemPostUpdate {
  public:
    MagnetometerPlugin ();
    ~MagnetometerPlugin () override;

    void Configure (const gz::sim::Entity &_entity,
                    const std::shared_ptr<const sdf::Element> &_sdf,
                    gz::sim::EntityComponentManager &_ecm,
                    gz::sim::EventManager & /*_eventMgr*/) override;

    void PreUpdate (const gz::sim::UpdateInfo &_info,
                    gz::sim::EntityComponentManager &_ecm) override;

    void PostUpdate (const gz::sim::UpdateInfo &_info,
                     const gz::sim::EntityComponentManager &_ecm) override;

  private:
    void addNoise (Eigen::Vector3d *magnetic_field, const double dt);
    void GroundtruthCallback (const sensor_msgs::msgs::Groundtruth &gt_msg);
    void getSdfParams (const std::shared_ptr<const sdf::Element> &sdf);

    gz::sim::Model model_{gz::sim::kNullEntity};
    gz::sim::Entity model_link_{gz::sim::kNullEntity};

    std::string mag_topic_;
    gz::transport::Node node;
    gz::transport::Node::Publisher pub_mag_;
    std::string gt_sub_topic_;

    double groundtruth_lat_rad_;
    double groundtruth_lon_rad_;

    sensor_msgs::msgs::MagneticField mag_message_;

    std::chrono::steady_clock::duration last_time_{0};
    std::chrono::steady_clock::duration last_pub_time_{0};
    unsigned int pub_rate_;
    double noise_density_;
    double random_walk_;
    double bias_correlation_time_;

    Eigen::Vector3d bias_;

    std::default_random_engine random_generator_;
    std::normal_distribution<double> standard_normal_distribution_;
};

} // namespace magnetometer_plugin

#endif // MAGNETOMETER_PLUGIN_HH_
