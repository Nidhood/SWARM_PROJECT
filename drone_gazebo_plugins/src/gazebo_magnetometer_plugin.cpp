/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be used to
 *    endorse or promote products derived from this software without specific
 *    prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 *LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
/**
 * @brief Magnetometer Plugin
 *
 * This plugin simulates magnetometer data
 *
 * Author: Elia Tarasov <elias.tarasov@gmail.com>
 */

#include "gazebo_magnetometer_plugin.hh"

#include <gz/plugin/Register.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/Name.hh>

GZ_ADD_PLUGIN (magnetometer_plugin::MagnetometerPlugin, gz::sim::System,
               magnetometer_plugin::MagnetometerPlugin::ISystemConfigure,
               magnetometer_plugin::MagnetometerPlugin::ISystemPreUpdate,
               magnetometer_plugin::MagnetometerPlugin::ISystemPostUpdate)

using namespace magnetometer_plugin;

MagnetometerPlugin::MagnetometerPlugin ()
    : groundtruth_lat_rad_ (0.0), groundtruth_lon_rad_ (0.0)
{
}

MagnetometerPlugin::~MagnetometerPlugin ()
{
}

void MagnetometerPlugin::getSdfParams (
    const std::shared_ptr<const sdf::Element> &sdf)
{
    if (sdf->HasElement ("pubRate")) {
        pub_rate_ = sdf->Get<unsigned int> ("pubRate");
    }
    else {
        pub_rate_ = kDefaultPubRate;
        gzerr
            << "[gazebo_magnetometer_plugin] Using default publication rate of "
            << pub_rate_ << " Hz\n";
    }

    if (sdf->HasElement ("noiseDensity")) {
        noise_density_ = sdf->Get<double> ("noiseDensity");
    }
    else {
        noise_density_ = kDefaultNoiseDensity;
        gzerr << "[gazebo_magnetometer_plugin] Using default noise density of "
              << noise_density_ << " (gauss)/sqrt(hz)\n";
    }

    if (sdf->HasElement ("randomWalk")) {
        random_walk_ = sdf->Get<double> ("randomWalk");
    }
    else {
        random_walk_ = kDefaultRandomWalk;
        gzerr << "[gazebo_magnetometer_plugin] Using default random walk of "
              << random_walk_ << " (gauss)*sqrt(hz)\n";
    }

    if (sdf->HasElement ("biasCorrelationTime")) {
        bias_correlation_time_ = sdf->Get<double> ("biasCorrelationTime");
    }
    else {
        bias_correlation_time_ = kDefaultBiasCorrelationTime;
        gzerr << "[gazebo_magnetometer_plugin] Using default bias correlation "
                 "time of "
              << bias_correlation_time_ << " s\n";
    }

    if (sdf->HasElement ("magTopic")) {
        mag_topic_ = sdf->Get<std::string> ("magTopic");
    }
    else {
        mag_topic_ = kDefaultMagnetometerTopic;
        gzerr
            << "[gazebo_magnetometer_plugin] Using default magnetometer topic "
            << mag_topic_ << "\n";
    }

    gt_sub_topic_ = "/groundtruth";
}

void MagnetometerPlugin::Configure (
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager & /*_eventMgr*/)
{
    getSdfParams (_sdf);

    pub_mag_ = this->node.Advertise<sensor_msgs::msgs::MagneticField> (
        "/world/drone_world/model/swarm_drone/link/base_link/sensor/"
        "magnetometer");
    node.Subscribe (gt_sub_topic_, &MagnetometerPlugin::GroundtruthCallback,
                    this);

    standard_normal_distribution_ = std::normal_distribution<double> (0.0, 1.0);

    bias_.setZero ();

    // Fill covariance matrix for the magnetic field measurement:
    for (int i = 0; i < 9; ++i) {
        if (i == 0 || i == 4 || i == 8) {
            mag_message_.add_magnetic_field_covariance (noise_density_ *
                                                        noise_density_);
        }
        else {
            mag_message_.add_magnetic_field_covariance (0.0);
        }
    }

    auto linkName = _sdf->Get<std::string> ("link_name");
    model_ = gz::sim::Model (_entity);
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

void MagnetometerPlugin::GroundtruthCallback (
    const sensor_msgs::msgs::Groundtruth &gt_msg)
{
    // Update groundtruth latitude and longitude (in radians)
    groundtruth_lat_rad_ = gt_msg.latitude_rad ();
    groundtruth_lon_rad_ = gt_msg.longitude_rad ();
}

void MagnetometerPlugin::addNoise (Eigen::Vector3d *magnetic_field,
                                   const double dt)
{
    assert (dt > 0.0);

    double tau = bias_correlation_time_;
    double sigma_d = 1 / sqrt (dt) * noise_density_;
    double sigma_b = random_walk_;
    double sigma_b_d =
        sqrt (-sigma_b * sigma_b * tau / 2.0 * (exp (-2.0 * dt / tau) - 1.0));
    double phi_d = exp (-dt / tau);
    for (int i = 0; i < 3; ++i) {
        bias_[i] =
            phi_d * bias_[i] +
            sigma_b_d * standard_normal_distribution_ (random_generator_);
        (*magnetic_field)[i] +=
            bias_[i] +
            sigma_d * standard_normal_distribution_ (random_generator_);
    }
}

void MagnetometerPlugin::PreUpdate (const gz::sim::UpdateInfo &_info,
                                    gz::sim::EntityComponentManager &_ecm)
{
    // No pre-update processing required.
}

void MagnetometerPlugin::PostUpdate (
    const gz::sim::UpdateInfo &_info,
    const gz::sim::EntityComponentManager &_ecm)
{
    const auto current_time = _info.simTime;
    const double dt =
        std::chrono::duration<double> (current_time - last_pub_time_).count ();
    if (dt > 1.0 / pub_rate_) {

        // Compute magnetic field from WMM2018 (values in 10^5 x nanoTesla in
        // n-frame)
        float declination_rad =
            get_mag_declination (groundtruth_lat_rad_ * 180.0 / M_PI,
                                 groundtruth_lon_rad_ * 180.0 / M_PI) *
            M_PI / 180.0;
        float inclination_rad =
            get_mag_inclination (groundtruth_lat_rad_ * 180.0 / M_PI,
                                 groundtruth_lon_rad_ * 180.0 / M_PI) *
            M_PI / 180.0;
        float strength_ga =
            0.01f * get_mag_strength (groundtruth_lat_rad_ * 180.0 / M_PI,
                                      groundtruth_lon_rad_ * 180.0 / M_PI);

        float H = strength_ga * cosf (inclination_rad);
        float Z = tanf (inclination_rad) * H;
        float X = H * cosf (declination_rad);
        float Y = H * sinf (declination_rad);

        gz::math::Vector3d magnetic_field_I (X, Y, Z);

        // Get current world pose from the model link
        const auto *pComp =
            _ecm.Component<gz::sim::components::WorldPose> (model_link_);
        const gz::math::Pose3d T_W_I = pComp->Data ();

        // Compute rotation from body to world frame (assuming q_ENU_to_NED and
        // q_FLU_to_FRD are defined)
        gz::math::Quaterniond q_body_to_world =
            q_ENU_to_NED * T_W_I.Rot () * q_FLU_to_FRD.Inverse ();

        // Transform magnetic field from inertial (n-frame) to body frame
        gz::math::Vector3d magnetic_field_B =
            q_body_to_world.RotateVectorReverse (magnetic_field_I);

        // Convert measured field to Eigen vector and add noise.
        Eigen::Vector3d measured_mag (magnetic_field_B.X (),
                                      magnetic_field_B.Y (),
                                      magnetic_field_B.Z ());
        addNoise (&measured_mag, dt);

        // Fill magnetometer message.
        mag_message_.set_time_usec (
            std::chrono::duration_cast<std::chrono::microseconds> (current_time)
                .count ());
        gz::msgs::Vector3d *magnetic_field = new gz::msgs::Vector3d ();
        magnetic_field->set_x (measured_mag[0]);
        magnetic_field->set_y (measured_mag[1]);
        magnetic_field->set_z (measured_mag[2]);
        mag_message_.set_allocated_magnetic_field (magnetic_field);

        last_pub_time_ = current_time;

        // Publish the magnetometer message.
        pub_mag_.Publish (mag_message_);
    }
}
