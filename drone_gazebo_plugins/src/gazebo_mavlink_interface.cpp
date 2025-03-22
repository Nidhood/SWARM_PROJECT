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

#include "gazebo_mavlink_interface.hh"

#include <gz/plugin/Register.hh>

GZ_ADD_PLUGIN (mavlink_interface::GazeboMavlinkInterface, gz::sim::System,
               mavlink_interface::GazeboMavlinkInterface::ISystemConfigure,
               mavlink_interface::GazeboMavlinkInterface::ISystemPreUpdate,
               mavlink_interface::GazeboMavlinkInterface::ISystemPostUpdate)

using namespace mavlink_interface;

GazeboMavlinkInterface::GazeboMavlinkInterface ()
    : input_offset_{}, input_scaling_{}, zero_position_disarmed_{},
      zero_position_armed_{}, input_index_{}
{
    mavlink_interface_ = std::make_shared<MavlinkInterface> ();
}

GazeboMavlinkInterface::~GazeboMavlinkInterface ()
{
    mavlink_interface_->close ();
}

void GazeboMavlinkInterface::Configure (
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm, gz::sim::EventManager &_em)
{
    namespace_.clear ();
    if (_sdf->HasElement ("robotNamespace")) {
        namespace_ = _sdf->Get<std::string> ("robotNamespace");
    }
    else {
        gzerr
            << "[gazebo_mavlink_interface] Please specify a robotNamespace.\n";
    }

    if (_sdf->HasElement ("protocol_version")) {
        protocol_version_ = _sdf->Get<float> ("protocol_version");
    }

    gazebo::getSdfParam<std::string> (_sdf, "motorSpeedCommandPubTopic",
                                      motor_velocity_reference_pub_topic_,
                                      motor_velocity_reference_pub_topic_);
    gazebo::getSdfParam<std::string> (_sdf, "gpsSubTopic", gps_sub_topic_,
                                      gps_sub_topic_);
    gazebo::getSdfParam<std::string> (_sdf, "visionSubTopic", vision_sub_topic_,
                                      vision_sub_topic_);
    gazebo::getSdfParam<std::string> (_sdf, "opticalFlowSubTopic",
                                      opticalFlow_sub_topic_,
                                      opticalFlow_sub_topic_);
    gazebo::getSdfParam<std::string> (_sdf, "irlockSubTopic", irlock_sub_topic_,
                                      irlock_sub_topic_);
    gazebo::getSdfParam<std::string> (_sdf, "magSubTopic", mag_sub_topic_,
                                      mag_sub_topic_);
    gazebo::getSdfParam<std::string> (_sdf, "baroSubTopic", baro_sub_topic_,
                                      baro_sub_topic_);
    groundtruth_sub_topic_ = "/groundtruth";

    // Set input_reference_ from inputs.control
    input_reference_.resize (n_out_max);

    // TODO: Parse input reference properly.
    input_scaling_.resize (n_out_max);
    input_scaling_ (0) = 1000;
    input_scaling_ (1) = 1000;
    input_scaling_ (2) = 1000;
    input_scaling_ (3) = 1000;

    if (_sdf->HasElement ("hil_mode")) {
        hil_mode_ = _sdf->Get<bool> ("hil_mode");
        mavlink_interface_->SetHILMode (hil_mode_);
    }

    if (_sdf->HasElement ("hil_state_level")) {
        hil_state_level_ = _sdf->Get<bool> ("hil_state_level");
        mavlink_interface_->SetHILStateLevel (hil_state_level_);
    }

    bool serial_enabled = false;
    if (_sdf->HasElement ("serialEnabled")) {
        serial_enabled = _sdf->Get<bool> ("serialEnabled");
        mavlink_interface_->SetSerialEnabled (serial_enabled);
    }

    bool use_tcp = false;
    if (!serial_enabled && _sdf->HasElement ("use_tcp")) {
        use_tcp = _sdf->Get<bool> ("use_tcp");
        mavlink_interface_->SetUseTcp (use_tcp);
    }
    gzmsg << "Connecting to PX4 SITL using "
          << (serial_enabled ? "serial" : (use_tcp ? "TCP" : "UDP")) << "\n";

    if (!hil_mode_ && _sdf->HasElement ("enable_lockstep")) {
        enable_lockstep_ = _sdf->Get<bool> ("enable_lockstep");
        mavlink_interface_->SetEnableLockstep (enable_lockstep_);
    }
    gzmsg << "Lockstep is " << (enable_lockstep_ ? "enabled" : "disabled")
          << "\n";

    // When running in lockstep, simulation speed factor can be set via env
    // variable.
    if (enable_lockstep_) {
        const char *speed_factor_str = std::getenv ("PX4_SIM_SPEED_FACTOR");
        if (speed_factor_str) {
            speed_factor_ = std::atof (speed_factor_str);
            if (!std::isfinite (speed_factor_) || speed_factor_ <= 0.0) {
                gzerr << "Invalid speed factor '" << speed_factor_str
                      << "', aborting\n";
                abort ();
            }
        }
        gzmsg << "Speed factor set to: " << speed_factor_ << "\n";
    }

    // Listen to Ctrl+C / SIGINT.
    sigIntConnection_ = _em.Connect<gz::sim::events::Stop> (
        std::bind (&GazeboMavlinkInterface::onSigInt, this));

    // Subscribe to messages from other plugins.
    node.Subscribe ("/imu", &GazeboMavlinkInterface::ImuCallback, this);
    node.Subscribe (
        "/world/quadcopter/model/X3/link/base_link/sensor/barometer",
        &GazeboMavlinkInterface::BarometerCallback, this);
    node.Subscribe (
        "/world/quadcopter/model/X3/link/base_link/sensor/magnetometer",
        &GazeboMavlinkInterface::MagnetometerCallback, this);
    node.Subscribe ("/world/quadcopter/model/X3/link/base_link/sensor/gps",
                    &GazeboMavlinkInterface::GpsCallback, this);

    if (_sdf->HasElement ("imu_rate")) {
        imu_update_interval_ = 1 / _sdf->Get<int> ("imu_rate");
    }

    if (_sdf->HasElement ("mavlink_addr")) {
        std::string mavlink_addr_str = _sdf->Get<std::string> ("mavlink_addr");
        if (mavlink_addr_str != "INADDR_ANY") {
            mavlink_interface_->SetMavlinkAddr (mavlink_addr_str);
        }
    }

    int mavlink_udp_port;
    if (_sdf->HasElement ("mavlink_udp_port")) {
        mavlink_udp_port = _sdf->Get<int> ("mavlink_udp_port");
    }
    mavlink_interface_->SetMavlinkUdpPort (mavlink_udp_port);

    int mavlink_tcp_port;
    if (_sdf->HasElement ("mavlink_tcp_port")) {
        mavlink_tcp_port = _sdf->Get<int> ("mavlink_tcp_port");
    }
    mavlink_interface_->SetMavlinkTcpPort (mavlink_tcp_port);

    if (_sdf->HasElement ("qgc_addr")) {
        std::string qgc_addr = _sdf->Get<std::string> ("qgc_addr");
        if (qgc_addr != "INADDR_ANY") {
            mavlink_interface_->SetGcsAddr (qgc_addr);
        }
    }
    if (_sdf->HasElement ("qgc_udp_port")) {
        int qgc_udp_port = _sdf->Get<int> ("qgc_udp_port");
        mavlink_interface_->SetGcsUdpPort (qgc_udp_port);
    }

    if (_sdf->HasElement ("sdk_addr")) {
        std::string sdk_addr = _sdf->Get<std::string> ("sdk_addr");
        if (sdk_addr != "INADDR_ANY") {
            mavlink_interface_->SetSdkAddr (sdk_addr);
        }
    }
    if (_sdf->HasElement ("sdk_udp_port")) {
        int sdk_udp_port = _sdf->Get<int> ("sdk_udp_port");
        mavlink_interface_->SetSdkUdpPort (sdk_udp_port);
    }

    if (serial_enabled_) {
        if (_sdf->HasElement ("serialDevice")) {
            std::string device = _sdf->Get<std::string> ("serialDevice");
            // Optionally set device.
        }
        if (_sdf->HasElement ("baudRate")) {
            int baudrate = _sdf->Get<int> ("baudRate");
            // Optionally set baud rate.
        }
    }

    mavlink_status_t *chan_state = mavlink_get_channel_status (MAVLINK_COMM_0);
    if (protocol_version_ == 2.0) {
        chan_state->flags &= ~(MAVLINK_STATUS_FLAG_OUT_MAVLINK1);
        gzmsg << "Using MAVLink protocol v2.0\n";
    }
    else if (protocol_version_ == 1.0) {
        chan_state->flags |= MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
        gzmsg << "Using MAVLink protocol v1.0\n";
    }
    else {
        gzerr << "Unknown protocol version! Using v" << protocol_version_
              << " by default\n";
    }
    mavlink_interface_->Load ();

    entity_ = _entity;
    model_ = gz::sim::Model (_entity);
    model_name_ = model_.Name (_ecm);
}

void GazeboMavlinkInterface::PreUpdate (const gz::sim::UpdateInfo &_info,
                                        gz::sim::EntityComponentManager &_ecm)
{
    const std::lock_guard<std::mutex> lock (last_imu_message_mutex_);

    if (!(previous_imu_seq_ % update_skip_factor_ == 0)) {
        return;
    }

    double dt;
    bool close_conn_ = false;

    if (hil_mode_) {
        mavlink_interface_->pollFromGcsAndSdk ();
    }
    else {
        mavlink_interface_->pollForMAVLinkMessages ();
    }

    SendSensorMessages (_info);
    SendGroundTruth ();

    if (close_conn_) {
        mavlink_interface_->close ();
    }

    handle_actuator_controls (_info);
    handle_control (dt);

    if (received_first_actuator_) {
        PublishRotorVelocities (_ecm, input_reference_);
    }
}

void GazeboMavlinkInterface::PostUpdate (
    const gz::sim::UpdateInfo &_info,
    const gz::sim::EntityComponentManager &_ecm)
{
    // Post-update processing (if needed).
}

void GazeboMavlinkInterface::ImuCallback (const gz::msgs::IMU &_msg)
{
    const std::lock_guard<std::mutex> lock (last_imu_message_mutex_);
    last_imu_message_ = _msg;
}

void GazeboMavlinkInterface::BarometerCallback (
    const sensor_msgs::msgs::Pressure &_msg)
{
    const std::lock_guard<std::mutex> lock (last_imu_message_mutex_);
    SensorData::Barometer baro_data;
    baro_data.temperature = _msg.temperature ();
    baro_data.abs_pressure = _msg.absolute_pressure ();
    baro_data.pressure_alt = _msg.pressure_altitude ();
    mavlink_interface_->UpdateBarometer (baro_data);
}

void GazeboMavlinkInterface::MagnetometerCallback (
    const sensor_msgs::msgs::MagneticField &_msg)
{
    const std::lock_guard<std::mutex> lock (last_imu_message_mutex_);
    SensorData::Magnetometer mag_data;
    mag_data.mag_b = Eigen::Vector3d (_msg.magnetic_field ().x (),
                                      _msg.magnetic_field ().y (),
                                      _msg.magnetic_field ().z ());
    mavlink_interface_->UpdateMag (mag_data);
}

void GazeboMavlinkInterface::GpsCallback (
    const sensor_msgs::msgs::SITLGps &_msg)
{
    const std::lock_guard<std::mutex> lock (last_imu_message_mutex_);
    SensorData::Gps gps_data;
    gps_data.time_utc_usec = _msg.time_utc_usec ();
    gps_data.fix_type = 3;
    gps_data.latitude_deg = _msg.latitude_deg () * 1e7;
    gps_data.longitude_deg = _msg.longitude_deg () * 1e7;
    gps_data.altitude = _msg.altitude () * 1000.0;
    gps_data.eph = _msg.eph () * 100.0;
    gps_data.epv = _msg.epv () * 100.0;
    gps_data.velocity = _msg.velocity () * 100.0;
    gps_data.velocity_north = _msg.velocity_north () * 100.0;
    gps_data.velocity_east = _msg.velocity_east () * 100.0;
    gps_data.velocity_down = -_msg.velocity_up () * 100.0;
    ignition::math::Angle cog (
        atan2 (_msg.velocity_east (), _msg.velocity_north ()));
    cog.Normalize ();
    gps_data.cog = static_cast<uint16_t> (gz::sim::GetDegrees360 (cog) * 100.0);
    gps_data.satellites_visible = 10;
    gps_data.id = 0;

    mavlink_interface_->SendGpsMessages (gps_data);
}

void GazeboMavlinkInterface::SendSensorMessages (
    const gz::sim::UpdateInfo &_info)
{
    gz::math::Quaterniond q_gr (last_imu_message_.orientation ().w (),
                                last_imu_message_.orientation ().x (),
                                last_imu_message_.orientation ().y (),
                                last_imu_message_.orientation ().z ());

    bool should_send_imu = false;
    if (!enable_lockstep_) {
        auto current_time = _info.simTime;
        double dt =
            std::chrono::duration<double> (current_time - last_imu_time_)
                .count ();
        if (imu_update_interval_ != 0 && dt >= imu_update_interval_) {
            should_send_imu = true;
            last_imu_time_ = current_time;
        }
    }

    int time_usec = std::chrono::duration_cast<std::chrono::duration<int>> (
                        _info.simTime * 1e6)
                        .count ();

    gz::math::Vector3d accel_b = q_FLU_to_FRD.RotateVector (
        gz::math::Vector3d (last_imu_message_.linear_acceleration ().x (),
                            last_imu_message_.linear_acceleration ().y (),
                            last_imu_message_.linear_acceleration ().z ()));

    gz::math::Vector3d gyro_b = q_FLU_to_FRD.RotateVector (
        gz::math::Vector3d (last_imu_message_.angular_velocity ().x (),
                            last_imu_message_.angular_velocity ().y (),
                            last_imu_message_.angular_velocity ().z ()));

    SensorData::Imu imu_data;
    imu_data.accel_b =
        Eigen::Vector3d (accel_b.X (), accel_b.Y (), accel_b.Z ());
    imu_data.gyro_b = Eigen::Vector3d (gyro_b.X (), gyro_b.Y (), gyro_b.Z ());
    mavlink_interface_->UpdateIMU (imu_data);

    mavlink_interface_->SendSensorMessages (time_usec);
}

void GazeboMavlinkInterface::SendGroundTruth ()
{
    gz::math::Quaterniond q_gr (last_imu_message_.orientation ().w (),
                                last_imu_message_.orientation ().x (),
                                last_imu_message_.orientation ().y (),
                                last_imu_message_.orientation ().z ());

    gz::math::Quaterniond q_gb = q_gr * q_FLU_to_FRD.Inverse ();
    gz::math::Quaterniond q_nb = q_ENU_to_NED * q_gb;

    gz::math::Vector3d vel_b;        // TODO: Obtener datos reales
    gz::math::Vector3d vel_n;        // TODO: Obtener datos reales
    gz::math::Vector3d omega_nb_b;   // TODO: Obtener datos reales
    gz::math::Vector3d accel_true_b; // TODO: Obtener datos reales

    mavlink_hil_state_quaternion_t hil_state_quat;
    hil_state_quat.attitude_quaternion[0] = q_nb.W ();
    hil_state_quat.attitude_quaternion[1] = q_nb.X ();
    hil_state_quat.attitude_quaternion[2] = q_nb.Y ();
    hil_state_quat.attitude_quaternion[3] = q_nb.Z ();

    hil_state_quat.rollspeed = omega_nb_b.X ();
    hil_state_quat.pitchspeed = omega_nb_b.Y ();
    hil_state_quat.yawspeed = omega_nb_b.Z ();

    hil_state_quat.lat = groundtruth_lat_rad * 180 / M_PI * 1e7;
    hil_state_quat.lon = groundtruth_lon_rad * 180 / M_PI * 1e7;
    hil_state_quat.alt = groundtruth_altitude * 1000;

    hil_state_quat.vx = vel_n.X () * 100;
    hil_state_quat.vy = vel_n.Y () * 100;
    hil_state_quat.vz = vel_n.Z () * 100;

    hil_state_quat.ind_airspeed = vel_b.X ();

    hil_state_quat.xacc = accel_true_b.X () * 1000;
    hil_state_quat.yacc = accel_true_b.Y () * 1000;
    hil_state_quat.zacc = accel_true_b.Z () * 1000;

    if (!hil_mode_ || (hil_mode_ && hil_state_level_)) {
        mavlink_message_t msg;
        mavlink_msg_hil_state_quaternion_encode_chan (1, 200, MAVLINK_COMM_0,
                                                      &msg, &hil_state_quat);
        mavlink_interface_->send_mavlink_message (&msg);
    }
}

void GazeboMavlinkInterface::handle_actuator_controls (
    const gz::sim::UpdateInfo &_info)
{
    bool armed = mavlink_interface_->GetArmedState ();
    last_actuator_time_ = _info.simTime;
    for (unsigned i = 0; i < n_out_max; i++) {
        input_index_[i] = i;
    }
    input_reference_.resize (n_out_max);
    Eigen::VectorXd actuator_controls =
        mavlink_interface_->GetActuatorControls ();
    if (actuator_controls.size () < n_out_max)
        return; // TODO: Manejar este caso correctamente.

    for (int i = 0; i < input_reference_.size (); i++) {
        if (armed) {
            input_reference_[i] =
                (actuator_controls[input_index_[i]] + input_offset_[i]) *
                    input_scaling_ (i) +
                zero_position_armed_[i];
        }
        else {
            input_reference_[i] = zero_position_disarmed_[i];
        }
    }
    received_first_actuator_ = mavlink_interface_->GetReceivedFirstActuator ();
}

void GazeboMavlinkInterface::handle_control (double _dt)
{
    // Configurar posiciones de joint (no implementado aquí)
}

bool GazeboMavlinkInterface::IsRunning ()
{
    return true; // TODO: Implementar correctamente
}

void GazeboMavlinkInterface::onSigInt ()
{
    mavlink_interface_->onSigInt ();
}

void GazeboMavlinkInterface::PublishRotorVelocities (
    gz::sim::EntityComponentManager &_ecm, const Eigen::VectorXd &_vels)
{
    if (_vels.size () != rotor_velocity_message_.velocity_size ()) {
        rotor_velocity_message_.mutable_velocity ()->Resize (_vels.size (), 0);
    }
    for (int i = 0; i < _vels.size (); ++i) {
        rotor_velocity_message_.set_velocity (i, _vels (i));
    }
    auto actuatorMsgComp =
        _ecm.Component<gz::sim::components::Actuators> (model_.Entity ());
    if (actuatorMsgComp) {
        auto compFunc = [] (const gz::msgs::Actuators &_a,
                            const gz::msgs::Actuators &_b) {
            return std::equal (_a.velocity ().begin (), _a.velocity ().end (),
                               _b.velocity ().begin ());
        };
        auto state =
            actuatorMsgComp->SetData (this->rotor_velocity_message_, compFunc)
                ? gz::sim::ComponentState::PeriodicChange
                : gz::sim::ComponentState::NoChange;
        _ecm.SetChanged (model_.Entity (),
                         gz::sim::components::Actuators::typeId, state);
    }
    else {
        _ecm.CreateComponent (
            model_.Entity (),
            gz::sim::components::Actuators (this->rotor_velocity_message_));
    }
}
