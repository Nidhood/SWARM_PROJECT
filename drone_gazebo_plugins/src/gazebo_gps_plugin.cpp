/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
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
 * @brief GPS Plugin
 *
 * This plugin publishes GPS and Groundtruth data to be used and propagated
 *
 * Authors: Amy Wagoner <arwagoner@gmail.com>
 *          Nuno Marques <nuno.marques@dronesolutions.io>
 */

#include "gazebo_gps_plugin.hh"
#include <gz/plugin/Register.hh>

GZ_ADD_PLUGIN (gps_plugin::GpsPlugin, gz::sim::System,
               gps_plugin::GpsPlugin::ISystemConfigure,
               gps_plugin::GpsPlugin::ISystemPreUpdate,
               gps_plugin::GpsPlugin::ISystemPostUpdate)

using namespace gps_plugin;

GpsPlugin::GpsPlugin ()
{
    pub_gps_ = this->node.Advertise<sensor_msgs::msgs::SITLGps> (
        "/world/quadcopter/model/X3/link/base_link/sensor/gps");
}

GpsPlugin::~GpsPlugin ()
{
}

void GpsPlugin::Configure (const gz::sim::Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           gz::sim::EntityComponentManager &_ecm,
                           gz::sim::EventManager &_em)
{
    // Use environment variables for home position if set.
    const char *env_lat = std::getenv ("PX4_HOME_LAT");
    const char *env_lon = std::getenv ("PX4_HOME_LON");
    const char *env_alt = std::getenv ("PX4_HOME_ALT");

    if (env_lat) {
        lat_home_ = std::stod (env_lat) * M_PI / 180.0;
        gzmsg << "[gps_plugin] Home latitude is set to " << std::stod (env_lat)
              << ".\n";
    }
    else if (_sdf->HasElement ("homeLatitude")) {
        double latitude;
        gazebo::getSdfParam<double> (_sdf, "homeLatitude", latitude, lat_home_);
        lat_home_ = latitude * M_PI / 180.0;
    }

    if (env_lon) {
        lon_home_ = std::stod (env_lon) * M_PI / 180.0;
        gzmsg << "[gps_plugin] Home longitude is set to " << std::stod (env_lon)
              << ".\n";
    }
    else if (_sdf->HasElement ("homeLongitude")) {
        double longitude;
        gazebo::getSdfParam<double> (_sdf, "homeLongitude", longitude,
                                     lon_home_);
        lon_home_ = longitude * M_PI / 180.0;
    }

    if (env_alt) {
        alt_home_ = std::stod (env_alt);
        gzmsg << "[gps_plugin] Home altitude is set to " << alt_home_ << ".\n";
    }
    else if (_sdf->HasElement ("homeAltitude")) {
        gazebo::getSdfParam<double> (_sdf, "homeAltitude", alt_home_,
                                     alt_home_);
    }

    // Get random walk in XY plane
    if (_sdf->HasElement ("gpsXYRandomWalk")) {
        gazebo::getSdfParam<double> (_sdf, "gpsXYRandomWalk",
                                     gps_xy_random_walk_,
                                     kDefaultGpsXYRandomWalk);
    }
    else {
        gzerr << "[gps_plugin] Using default random walk in XY plane: "
              << kDefaultGpsXYRandomWalk << "\n";
    }

    // Get random walk in Z
    if (_sdf->HasElement ("gpsZRandomWalk")) {
        gazebo::getSdfParam<double> (_sdf, "gpsZRandomWalk", gps_z_random_walk_,
                                     kDefaultGpsZRandomWalk);
    }
    else {
        gzerr << "[gps_plugin] Using default random walk in Z: "
              << kDefaultGpsZRandomWalk << "\n";
    }

    // Get position noise density in XY plane
    if (_sdf->HasElement ("gpsXYNoiseDensity")) {
        gazebo::getSdfParam<double> (_sdf, "gpsXYNoiseDensity",
                                     gps_xy_noise_density_,
                                     kDefaultGpsXYNoiseDensity);
    }
    else {
        gzerr
            << "[gps_plugin] Using default position noise density in XY plane: "
            << kDefaultGpsXYNoiseDensity << "\n";
    }

    // Get position noise density in Z
    if (_sdf->HasElement ("gpsZNoiseDensity")) {
        gazebo::getSdfParam<double> (_sdf, "gpsZNoiseDensity",
                                     gps_z_noise_density_,
                                     kDefaultGpsZNoiseDensity);
    }
    else {
        gzerr << "[gps_plugin] Using default position noise density in Z: "
              << kDefaultGpsZNoiseDensity << "\n";
    }

    // Get velocity noise density in XY plane
    if (_sdf->HasElement ("gpsVXYNoiseDensity")) {
        gazebo::getSdfParam<double> (_sdf, "gpsVXYNoiseDensity",
                                     gps_vxy_noise_density_,
                                     kDefaultGpsVXYNoiseDensity);
    }
    else {
        gzerr
            << "[gps_plugin] Using default velocity noise density in XY plane: "
            << kDefaultGpsVXYNoiseDensity << "\n";
    }

    // Get velocity noise density in Z
    if (_sdf->HasElement ("gpsVZNoiseDensity")) {
        gazebo::getSdfParam<double> (_sdf, "gpsVZNoiseDensity",
                                     gps_vz_noise_density_,
                                     kDefaultGpsVZNoiseDensity);
    }
    else {
        gzerr << "[gps_plugin] Using default velocity noise density in Z: "
              << kDefaultGpsVZNoiseDensity << "\n";
    }

    // Get update rate
    if (_sdf->HasElement ("update_rate")) {
        gazebo::getSdfParam<double> (_sdf, "update_rate", update_rate_,
                                     kDefaultUpdateRate);
    }
    else {
        update_rate_ = kDefaultUpdateRate;
        gzerr << "[gps_plugin] Using default update rate of "
              << kDefaultUpdateRate << "hz \n";
    }

    auto linkName = _sdf->Get<std::string> ("link_name");
    model_ = gz::sim::Model (_entity);
    // Get link entity
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

void GpsPlugin::PreUpdate (const gz::sim::UpdateInfo &_info,
                           gz::sim::EntityComponentManager &_ecm)
{
    // No pre-update processing is performed.
}

void GpsPlugin::PostUpdate (const gz::sim::UpdateInfo &_info,
                            const gz::sim::EntityComponentManager &_ecm)
{
    const std::chrono::steady_clock::duration current_time = _info.simTime;
    const double dt =
        std::chrono::duration<double> (current_time - last_pub_time_).count ();

    if (dt > 1.0 / update_rate_) {
        const auto *pComp =
            _ecm.Component<gz::sim::components::WorldPose> (model_link_);
        const gz::math::Pose3d T_W_I = pComp->Data ();
        // Use the model's world position for GPS
        const gz::math::Vector3d &pos_W_I = T_W_I.Pos ();
        const gz::math::Quaterniond &att_W_I = T_W_I.Rot ();

        // Use the model's world linear velocity for GPS velocity.
        const auto *vComp =
            _ecm.Component<gz::sim::components::WorldLinearVelocity> (
                model_link_);
        gz::math::Vector3d velocity_current_W = vComp->Data ();
        gz::math::Vector3d velocity_current_W_xy = velocity_current_W;
        velocity_current_W_xy.Z () = 0;

        // Update noise parameters if gps_noise_ is enabled.
        if (gps_noise_) {
            noise_gps_pos_.X () =
                gps_xy_noise_density_ * sqrt (dt) * randn_ (rand_);
            noise_gps_pos_.Y () =
                gps_xy_noise_density_ * sqrt (dt) * randn_ (rand_);
            noise_gps_pos_.Z () =
                gps_z_noise_density_ * sqrt (dt) * randn_ (rand_);
            noise_gps_vel_.X () =
                gps_vxy_noise_density_ * sqrt (dt) * randn_ (rand_);
            noise_gps_vel_.Y () =
                gps_vxy_noise_density_ * sqrt (dt) * randn_ (rand_);
            noise_gps_vel_.Z () =
                gps_vz_noise_density_ * sqrt (dt) * randn_ (rand_);
            random_walk_gps_.X () =
                gps_xy_random_walk_ * sqrt (dt) * randn_ (rand_);
            random_walk_gps_.Y () =
                gps_xy_random_walk_ * sqrt (dt) * randn_ (rand_);
            random_walk_gps_.Z () =
                gps_z_random_walk_ * sqrt (dt) * randn_ (rand_);
        }
        else {
            noise_gps_pos_.X () = 0.0;
            noise_gps_pos_.Y () = 0.0;
            noise_gps_pos_.Z () = 0.0;
            noise_gps_vel_.X () = 0.0;
            noise_gps_vel_.Y () = 0.0;
            noise_gps_vel_.Z () = 0.0;
            random_walk_gps_.X () = 0.0;
            random_walk_gps_.Y () = 0.0;
            random_walk_gps_.Z () = 0.0;
        }

        // Integrate GPS bias
        gps_bias_.X () +=
            random_walk_gps_.X () * dt - gps_bias_.X () / gps_corellation_time_;
        gps_bias_.Y () +=
            random_walk_gps_.Y () * dt - gps_bias_.Y () / gps_corellation_time_;
        gps_bias_.Z () +=
            random_walk_gps_.Z () * dt - gps_bias_.Z () / gps_corellation_time_;

        // Reproject position with noise into geographic coordinates.
        auto pos_with_noise = pos_W_I + noise_gps_pos_ + gps_bias_;
        auto latlon =
            reproject (pos_with_noise, lat_home_, lon_home_, alt_home_);

        // Fill SITLGps message.
        sensor_msgs::msgs::SITLGps gps_msg;
        gps_msg.set_time_usec (
            std::chrono::duration_cast<std::chrono::microseconds> (current_time)
                .count ());
        gps_msg.set_time_utc_usec (
            std::chrono::duration_cast<std::chrono::microseconds> (current_time)
                .count ());
        // TODO: Add start time if needed.

        gps_msg.set_latitude_deg (latlon.first * 180.0 / M_PI);
        gps_msg.set_longitude_deg (latlon.second * 180.0 / M_PI);
        gps_msg.set_altitude (pos_W_I.Z () + alt_home_ - noise_gps_pos_.Z () +
                              gps_bias_.Z ());

        std_xy_ = 1.0;
        std_z_ = 1.0;
        gps_msg.set_eph (std_xy_);
        gps_msg.set_epv (std_z_);

        gps_msg.set_velocity_east (velocity_current_W.X () +
                                   noise_gps_vel_.Y ());
        gps_msg.set_velocity (velocity_current_W_xy.Length ());
        gps_msg.set_velocity_north (velocity_current_W.Y () +
                                    noise_gps_vel_.X ());
        gps_msg.set_velocity_up (velocity_current_W.Z () - noise_gps_vel_.Z ());

        pub_gps_.Publish (gps_msg);
        last_pub_time_ = current_time;
    }
}
