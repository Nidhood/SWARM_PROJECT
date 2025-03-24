#ifndef GPS_PLUGIN_HH_
#define GPS_PLUGIN_HH_

#include <random>

#include <gz/msgs/float.pb.h> // Biblioteca para mensajes de Gazebo.
#include <gz/sim/Model.hh>    // Biblioteca para el modelo de simulación.
#include <gz/sim/System.hh>   // Biblioteca para el sistema de simulación.
#include <gz/sim/Util.hh>     // Biblioteca para utilidades de simulación.
#include <gz/sim/World.hh>    // Biblioteca para el mundo de simulación.
#include <gz/sim/components/LinearVelocity.hh> // Biblioteca para la velocidad lineal.
#include <gz/sim/components/Pose.hh> // Biblioteca para la pose del modelo.
#include <gz/sim/config.hh>          // Configuración de simulación.

#include <gz/transport/Node.hh>      // Nodo de transporte para comunicación.
#include <gz/transport/Publisher.hh> // Publicador de transporte.

#include <memory>
#include <rclcpp/rclcpp.hpp> // Library for ROS2 cpp
#include <std_msgs/msg/float64.hpp>
#include <string>

#include "gz/sim/components/AngularVelocity.hh" // Velocidad angular.
#include "gz/sim/components/LinearVelocity.hh" // (ya incluido arriba, pero se deja para referencia)
#include "gz/sim/components/Pose.hh"  // (ya incluido)
#include "gz/sim/components/World.hh" // (ya incluido)
#include <gz/common/Console.hh>       // Consola para mensajes.
#include <gz/math/Pose3.hh>           // Para pose 3D.
#include <gz/math/Vector3.hh>         // Para vectores 3D.
#include <gz/plugin/Register.hh>      // Registro del plugin.
#include <gz/sim/Util.hh> // Utilidades de simulación (ya incluido).

#include <gz/sim/Joint.hh> // Biblioteca para articulaciones.
#include <trajectory_msgs/msg/joint_trajectory_point.hpp> // Mensajes ROS2 para puntos de trayectoria.

#include "drone_msgs/msg/sitl_gps.hpp" // Mensaje ROS2 para GPS.
#include <gz/msgs/joint_trajectory.pb.h> // Mensaje de trayectoria de articulaciones.
#include <gz/msgs/joint_trajectory_point.pb.h> // Mensaje de punto de trayectoria.
#include <gz/transport/AdvertiseOptions.hh> // Opciones para la publicidad de mensajes.
#include <rclcpp/rclcpp.hpp>                // Library for ROS2 cpp

#include "SITLGps.pb.h"
#include "common.hh"

// Define el plugin usando el macro de exportación propio de Gazebo Sim.
namespace gps_plugin {
class GZ_SIM_VISIBLE GpsPlugin : public gz::sim::System,
                                 public gz::sim::ISystemConfigure,
                                 public gz::sim::ISystemPreUpdate,
                                 public gz::sim::ISystemPostUpdate {
  public:
    GpsPlugin ();
    ~GpsPlugin () override;

    void Configure (const gz::sim::Entity &_entity,
                    const std::shared_ptr<const sdf::Element> &_sdf,
                    gz::sim::EntityComponentManager &_ecm,
                    gz::sim::EventManager & /*_eventMgr*/) override;

    void PreUpdate (const gz::sim::UpdateInfo &_info,
                    gz::sim::EntityComponentManager &_ecm) override;

    void PostUpdate (const gz::sim::UpdateInfo &_info,
                     const gz::sim::EntityComponentManager &_ecm) override;

  private:
    std::chrono::steady_clock::duration last_pub_time_{0};

    std::string namespace_;
    std::string gps_id_;
    std::default_random_engine random_generator_;
    std::normal_distribution<float> standard_normal_distribution_;

    bool gps_noise_{false};

    std::string model_name_;

    gz::sim::Model model_{gz::sim::kNullEntity};
    gz::sim::Entity model_link_{gz::sim::kNullEntity};

    gz::transport::Node node;
    gz::transport::Node::Publisher pub_gps_;
    std::shared_ptr<rclcpp::Node> ros_node_;
    std::shared_ptr<rclcpp::Publisher<drone_msgs::msg::SITLGps>> pub_gps_ros2_;
    std::string topic_gps_ros2_{"gps"};

    std::string gps_topic_;
    double update_rate_{1.0};

    std::mutex data_mutex_;

    // Home defaults to Zurich Irchel Park
    double lat_home_ = kDefaultHomeLatitude;
    double lon_home_ = kDefaultHomeLongitude;
    double alt_home_ = kDefaultHomeAltitude;
    double world_latitude_ = 0.0;
    double world_longitude_ = 0.0;
    double world_altitude_ = 0.0;

    // gps delay related
    static constexpr double gps_delay_ = 0.12; // 120 ms
    static constexpr int gps_buffer_size_max_ = 1000;
    std::queue<sensor_msgs::msgs::SITLGps> gps_delay_buffer_;
    drone_msgs::msg::SITLGps ros_gps_msg_;

    gz::math::Vector3d gps_bias_;
    gz::math::Vector3d noise_gps_pos_;
    gz::math::Vector3d noise_gps_vel_;
    gz::math::Vector3d random_walk_gps_;
    gz::math::Vector3d gravity_W_;
    gz::math::Vector3d velocity_prev_W_;

    // gps noise parameters
    double std_xy_; // meters
    double std_z_;  // meters
    std::default_random_engine rand_;
    std::normal_distribution<float> randn_;
    static constexpr const double gps_corellation_time_ = 60.0; // s
    double gps_xy_random_walk_;
    double gps_z_random_walk_;
    double gps_xy_noise_density_;
    double gps_z_noise_density_;
    double gps_vxy_noise_density_;
    double gps_vz_noise_density_;
};
} // namespace gps_plugin

#endif
