/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
/**
 * @brief Barometer Plugin
 *
 * This plugin simulates barometer data
 *
 * Author: Elia Tarasov <elias.tarasov@gmail.com>
 */

#include "gazebo_barometer_plugin.hh"

#include <gz/plugin/Register.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/Name.hh>

GZ_ADD_PLUGIN (barometer_plugin::BarometerPlugin, gz::sim::System,
               barometer_plugin::BarometerPlugin::ISystemConfigure,
               barometer_plugin::BarometerPlugin::ISystemPreUpdate,
               barometer_plugin::BarometerPlugin::ISystemPostUpdate)

using namespace barometer_plugin;

BarometerPlugin::BarometerPlugin ()
{

    // Publish the pressure message using the Gazebo Transport API.
    pub_baro_ = this->node.Advertise<sensor_msgs::msgs::Pressure> (
        "/world/drone_world/model/swarm_drone/link/base_link/sensor/barometer");

    // Publish the pressure message using ROS2.
    if (!rclcpp::ok ()) {
        rclcpp::init (0, nullptr);
    }
    ros_node_ = std::make_shared<rclcpp::Node> ("barometer_plugin");
    pub_baro_ros2_ = ros_node_->create_publisher<drone_msgs::msg::Pressure> (
        "barometer", 10);
}

BarometerPlugin::~BarometerPlugin ()
{
}

void BarometerPlugin::Configure (
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm, gz::sim::EventManager &_em)
{
    auto linkName = _sdf->Get<std::string> ("link_name");
    model_ = gz::sim::Model (_entity);
    // Obtener la entidad del link mediante su nombre
    model_link_ = model_.LinkByName (_ecm, linkName);

    if (!_ecm.EntityHasComponentType (model_link_,
                                      gz::sim::components::WorldPose::typeId)) {
        _ecm.CreateComponent (model_link_, gz::sim::components::WorldPose ());
    }
    if (!_ecm.EntityHasComponentType (
            model_link_, gz::sim::components::WorldLinearVelocity::typeId)) {
        _ecm.CreateComponent (model_link_,
                              gz::sim::components::WorldLinearVelocity ());
    }
}

void BarometerPlugin::PreUpdate (const gz::sim::UpdateInfo &_info,
                                 gz::sim::EntityComponentManager &_ecm)
{
    // Does not perform any preprocessing in this plugin.
}

void BarometerPlugin::PostUpdate (const gz::sim::UpdateInfo &_info,
                                  const gz::sim::EntityComponentManager &_ecm)
{
    const std::chrono::steady_clock::duration current_time = _info.simTime;
    const double dt =
        std::chrono::duration<double> (current_time - last_pub_time_).count ();
    if (dt > 1.0 / pub_rate_) {
        // Obtener la pose del modelo al que está adjunto el plugin
        const auto *pComp =
            _ecm.Component<gz::sim::components::WorldPose> (model_link_);
        const gz::math::Pose3d pose_model_world = pComp->Data ();

        // Pose en el marco local (se toma solo la componente Z relativa a la
        // posición inicial)
        gz::math::Pose3d pose_model;
        pose_model.Pos ().Z () =
            pose_model_world.Pos ().Z () - pose_model_start_.Pos ().Z ();
        const float pose_n_z =
            -pose_model.Pos ()
                 .Z (); // Conversión de ENU a NED para la componente Z

        // Cálculo de la presión absoluta usando un modelo ISA para la
        // troposfera (válido hasta 11 km sobre MSL)
        const float lapse_rate =
            0.0065f; // Reducción de temperatura con altitud (K/m)
        const float temperature_msl = 288.0f; // Temperatura a nivel del mar (K)
        const float alt_msl = static_cast<float> (alt_home_) - pose_n_z;
        const float temperature_local = temperature_msl - lapse_rate * alt_msl;
        const float pressure_ratio =
            powf (temperature_msl / temperature_local, 5.256f);
        const float pressure_msl = 101325.0f; // Presión a nivel del mar (Pa)
        const float absolute_pressure = pressure_msl / pressure_ratio;

        // Genera ruido gaussiano usando la transformación polar de Box-Muller
        double y1;
        {
            double x1, x2, w;
            if (!baro_rnd_use_last_) {
                do {
                    x1 = 2.0 *
                             standard_normal_distribution_ (random_generator_) -
                         1.0;
                    x2 = 2.0 *
                             standard_normal_distribution_ (random_generator_) -
                         1.0;
                    w = x1 * x1 + x2 * x2;
                } while (w >= 1.0);
                w = sqrt ((-2.0 * log (w)) / w);
                // Se calculan dos valores; el segundo se utiliza en la
                // siguiente actualización
                y1 = x1 * w;
                baro_rnd_y2_ = x2 * w;
                baro_rnd_use_last_ = true;
            }
            else {
                // Usar el valor almacenado de la actualización anterior
                y1 = baro_rnd_y2_;
                baro_rnd_use_last_ = false;
            }
        }

        // Aplicar ruido y deriva
        const float abs_pressure_noise =
            1.0f * static_cast<float> (y1); // 1 Pa RMS de ruido
        baro_drift_pa_ += baro_drift_pa_per_sec_ * dt;
        const float absolute_pressure_noisy =
            absolute_pressure + abs_pressure_noise + baro_drift_pa_;

        // Convertir a hPa
        const float absolute_pressure_noisy_hpa =
            absolute_pressure_noisy * 0.01f;
        baro_msg_.set_absolute_pressure (absolute_pressure_noisy_hpa);
        ros_baro_msg_.absolute_pressure = absolute_pressure_noisy_hpa;
        // Calcular la densidad usando un modelo ISA para la troposfera
        // (válido hasta 11 km sobre MSL)
        const float density_ratio =
            powf (temperature_msl / temperature_local, 4.256f);
        const float rho = 1.225f / density_ratio;
        // Calcular la altitud de presión incluyendo el efecto del ruido
        baro_msg_.set_pressure_altitude (alt_msl -
                                         (abs_pressure_noise + baro_drift_pa_) /
                                             (gravity_W_.Length () * rho));
        ros_baro_msg_.pressure_altitude =
            alt_msl - (abs_pressure_noise + baro_drift_pa_) /
                          (gravity_W_.Length () * rho);

        // Calcular la temperatura en Celsius
        baro_msg_.set_temperature (temperature_local - 273.0f);
        ros_baro_msg_.temperature = temperature_local - 273.0f;

        // Rellenar el mensaje de barómetro
        baro_msg_.set_time_usec (
            std::chrono::duration_cast<std::chrono::microseconds> (current_time)
                .count ());
        ros_baro_msg_.time_usec =
            std::chrono::duration_cast<std::chrono::microseconds> (current_time)
                .count ();

        last_pub_time_ = current_time;

        // Publicar el mensaje de barómetro
        pub_baro_.Publish (baro_msg_);
        pub_baro_ros2_->publish (ros_baro_msg_);
    }
}
