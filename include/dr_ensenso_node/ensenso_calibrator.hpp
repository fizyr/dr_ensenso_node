#pragma once

#include <dr_ensenso_msgs/InitializeCalibration.h>
#include <dr_msgs/GetPose.h>
#include <dr_ensenso/calibration_param.hpp>
#include <dr_msgs/SendPose.h>
#include <dr_ensenso_msgs/FinalizeCalibration.h>
#include <dr_ros/service_client.hpp>
#include <dr_ros/node.hpp>
#include <dr_eigen/ros.hpp>

#include <estd/result.hpp>

#include <ros/node_handle.h>

#include <Eigen/Eigen>

namespace dr {

class EnsensoCalibratorNode: public Node {
public:
	struct CalibrationResult {
		/// Calibrated pose of the camera in either moving_frame or fixed_frame (depending on camera_moving).
		Eigen::Isometry3d camera_pose;

		/// Calibrated pose of the pattern in either moving_frame or fixed_frame (depending on camera_moving).
		Eigen::Isometry3d pattern_pose;

		/// The residual error of the calibration.
		double residual_error;
	};

private:
	/// List of ros services.
	struct Services {
		/// Service for initializing a calibration sequence.
		dr::ServiceClient<dr_ensenso_msgs::InitializeCalibration> initialize_calibration;

		/// Service for recording a calibration pattern.
		dr::ServiceClient<dr_msgs::SendPose> record_calibration;

		/// Service for finalizing a calibration sequence.
		dr::ServiceClient<dr_ensenso_msgs::FinalizeCalibration> finalize_calibration;

		/// Service for getting the calibration on the camera.
		dr::ServiceClient<dr_msgs::GetPose> get_calibration;
	} services_;

	bool store_calibration_ = true;

public:
	EnsensoCalibratorNode(
		std::string const & initialize_calibration_service, ///< Service name for the initialization of the calibration.
		std::string const & record_calibration_service,     ///< Service name for the detect calibration pattern service.
		std::string const & finalize_calibration_service,   ///< Service name for finalizing the calibration.
		std::string const & get_calibration_service,        ///< Service name for getting the calibration on the camera.
		bool wait_for_services = false,                     ///< If true, waits for the services to come alive.
		bool store_calibration = true                       ///< If true, stores the calibration in the Ensenso.
	);

	/// Initializes a calibration sequence, clearing any state from previous calibration sequences.
	estd::result<void, estd::error> initializeCalibration(InitializeCalibrationConfig const & config);

	/// Records a single calibration pattern.
	estd::result<void, estd::error> recordCalibration(Eigen::Isometry3d const & robot_pose);

	/// Finalizes the calibration, returning the result.
	estd::result<CalibrationResult, estd::error> finalizeCalibration();

	/// Retrieves the calibration from the camera, returning the result.
	estd::result<Eigen::Isometry3d, estd::error> getCalibration();
};

}
