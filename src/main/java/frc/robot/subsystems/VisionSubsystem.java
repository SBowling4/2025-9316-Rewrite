package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.util.constants.Constants.VisionConstants;

public class VisionSubsystem extends SubsystemBase {

    private PhotonCamera camera;
    private PhotonPoseEstimator photonPoseEstimator;
    private PhotonPipelineResult result;
    
    // PID controllers for different axes of movement
    public PIDController rotController = new PIDController(VisionConstants.VR_Kp, VisionConstants.VR_Ki, VisionConstants.VR_Kd);
    public PIDController driveControllerY = new PIDController(VisionConstants.VY_Kp, VisionConstants.VY_Ki, VisionConstants.VY_Kd);
    public PIDController driveControllerX = new PIDController(VisionConstants.VX_Kp, VisionConstants.VX_Ki, VisionConstants.VX_Kd);

    private AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    
    // Transform from camera to robot center (measurements in meters)
    public static final Transform3d CAMERA_TO_ROBOT = 
        new Transform3d(new Translation3d(0.05, 0.0, 0.1), new Rotation3d(0, 0, 0));

    private CommandSwerveDrivetrain drivetrain = RobotContainer.getDrivetrain();
    
    public VisionSubsystem() {
        // Initialize camera with name from constants
        camera = new PhotonCamera(VisionConstants.TARGET_CAMERA);
        
        // Initialize pose estimator with field layout and camera transform
        photonPoseEstimator = new PhotonPoseEstimator(
                aprilTagFieldLayout,
                PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
                CAMERA_TO_ROBOT);
        
        // Get initial result
        result = getLatestResult().orElse(new PhotonPipelineResult());

        // Add debug info to Shuffleboard
        Shuffleboard.getTab("Vision").addBoolean("Camera Connected", () -> camera.isConnected());
        Shuffleboard.getTab("Vision").addBoolean("Has Target", () -> hasTarget());
        Shuffleboard.getTab("Vision").addNumber("Target Yaw", () -> getYaw().orElse(0.0));
        Shuffleboard.getTab("Vision").addNumber("Target Range", () -> getRange().orElse(0.0));
        
        // Add PIDs to Shuffleboard for tuning
        Shuffleboard.getTab("Vision").add("Rotation PID", rotController);
        Shuffleboard.getTab("Vision").add("Y PID", driveControllerY);
        Shuffleboard.getTab("Vision").add("X PID", driveControllerX);
    }

    /**
     * Gets the estimated robot pose based on vision
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return photonPoseEstimator.update(result);
    }

    // In VisionSubsystem.java, add a method to get the tag's rotation relative to the camera
    public Optional<Double> getTagYaw() {
        if (hasTarget()) {
            // This gives us the yaw of the tag, which tells us the tag's orientation
            return Optional.of(result.getBestTarget().getBestCameraToTarget().getRotation().getZ());
        } else {
            return Optional.empty();
        }
    }

    // Add a method to calculate rotation needed for parallel alignment
    public double calculateParallelRotationPower(double defaultRotation) {
        if (hasTarget()) {
            Optional<Double> tagYaw = getTagYaw();
            if (tagYaw.isPresent()) {
                double targetAngle;
                // To align parallel, we want our rotation to be 90 degrees (π/2 radians) 
                // offset from the tag's facing direction
                if(tagYaw.get() > 0) {
                    targetAngle = 90;
                } else {
                    targetAngle = -90;
                }
                
                // We still want to use the yaw to the target as our current angle,
                // as we're calculating the difference between where we're pointed and where we want to point
                return -rotController.calculate(getYaw().get(), targetAngle);
            }
        }
        return defaultRotation;
    }

    /**
     * Checks if the camera sees any AprilTags
     */
    public boolean hasTarget() {
        return result.hasTargets();
    }

    /**
     * Gets the yaw (horizontal angle) to the best target
     */
    public Optional<Double> getYaw() {
        if (hasTarget()) {
            return Optional.of(result.getBestTarget().getYaw());
        } else {
            return Optional.empty();
        }
    }

    /**
     * Gets the pitch (vertical angle) to the best target
     */
    public Optional<Double> getPitch() {
        if (hasTarget()) {
            return Optional.of(result.getBestTarget().getPitch());
        } else {
            return Optional.empty();
        }
    }

    /**
     * Gets the range (distance in meters) to the best target
     */
    public Optional<Double> getRange() {
        if (hasTarget()) {
            return Optional.of(result.getBestTarget().getBestCameraToTarget().getTranslation().getX());
        } else {
            return Optional.empty();
        }
    }

    /**
     * Gets the horizontal offset (Y in meters) to the best target
     */
    public Optional<Double> getHorizontalOffset() {
        if (hasTarget()) {
            return Optional.of(result.getBestTarget().getBestCameraToTarget().getTranslation().getY());
        } else {
            return Optional.empty();
        }
    }

    /**
     * Calculates the rotation power needed to align with a target
     * 
     * @param defaultRotation Value to use when no target is visible
     * @param enableVision Whether to use vision for alignment
     * @return The calculated rotation power
     */
    public double calculateRotationPower(double defaultRotation, boolean enableVision) {
        if (hasTarget() && enableVision) {
            return -rotController.calculate(getYaw().get(), 0.0);
        } else {
            return defaultRotation;
        }
    }

    /**
     * Calculates the forward/backward power needed to reach desired distance from target
     * 
     * @param defaultPower Value to use when no target is visible
     * @param targetDistance Desired distance from target in meters
     * @param enableVision Whether to use vision for alignment
     * @return The calculated X power
     */
    public double calculateXPower(double defaultPower, double targetDistance, boolean enableVision) {
        if (hasTarget() && enableVision) {
            System.out.println("Xpower:" + driveControllerX.calculate(getRange().get(), targetDistance));
            return driveControllerX.calculate(getRange().get(), targetDistance);
        } else {
            return defaultPower;
        }
    }

    /**
     * Calculates the left/right power needed to center with the target
     * 
     * @param defaultPower Value to use when no target is visible
     * @param targetOffset Desired horizontal offset (usually 0)
     * @param enableVision Whether to use vision for alignment
     * @return The calculated Y power
     */
    public double calculateYPower(double defaultPower, double targetOffset) {
        if (hasTarget()) {
            // Use the yaw for centering - negative because positive yaw means target is to the right
            System.out.println("Ypower:" + driveControllerY.calculate(getYaw().get(), -targetOffset));
            return driveControllerY.calculate(getYaw().get(), -targetOffset);
        } else {
            return defaultPower;
        }
    }

    private Optional<PhotonPipelineResult> getLatestResult() {
        var allResults = camera.getAllUnreadResults();
        int size = allResults.size();
        return size > 0 ? Optional.of(allResults.get(size - 1)) : Optional.empty();
    }

    private PhotonPipelineResult clearEvilResults(PhotonPipelineResult result) {
        List<PhotonTrackedTarget> evilTargets = new ArrayList<>();

        for (PhotonTrackedTarget target : result.getTargets()) {
            var tagPose = aprilTagFieldLayout.getTagPose(target.getFiducialId());
            if (tagPose.isEmpty()) {
                evilTargets.add(target);
                continue;
            }
            var distance = PhotonUtils.getDistanceToPose(drivetrain.getPose(), tagPose.get().toPose2d());

            if (target.getPoseAmbiguity() > .2 || distance > VisionConstants.MAX_DISTANCE) {
                evilTargets.add(target);
            }
        }

        result.targets.removeAll(evilTargets);

        return result;
    }

    @Override
    public void periodic() {
        if (!camera.isConnected()) return;

        var lastResult = result;
        result = getLatestResult().orElse(lastResult);
        if (!result.hasTargets()) return;

        var poseResult = result;
        poseResult = clearEvilResults(poseResult);
        
        Optional<EstimatedRobotPose> estimatedRobotPose = photonPoseEstimator.update(result);
        if (estimatedRobotPose.isEmpty()) return;

        drivetrain.addVisionMeasurement(estimatedRobotPose.get().estimatedPose.toPose2d(), estimatedRobotPose.get().timestampSeconds);
    }
}