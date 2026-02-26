package frc.robot.subsystems.Vision;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.posestimator.PoseEstimator;
import frc.lib.util.LimelightHelpers;
import frc.robot.RobotState;
import lombok.Getter;

public class Vision extends SubsystemBase {

    private final RobotState robotState;

    @Getter
    private PoseEstimator.VisionPoseObservation lastVisionObservation = new PoseEstimator.VisionPoseObservation(0, Pose2d.kZero, 0, 0);

    public Vision(RobotState robotState) {
        this.robotState = robotState;
    }

    public Pose2d updateFromMegaTag1() {
        LimelightHelpers.PoseEstimate limelightMeasurementMT1 = LimelightHelpers.getBotPoseEstimate_wpiRed("");
        return limelightMeasurementMT1.pose;
    }
    
    
    @Override
    public void periodic() {
//        double tx = LimelightHelpers.getTX("");  // Horizontal offset from crosshair to target in degrees
//        double ty = LimelightHelpers.getTY("");  // Vertical offset from crosshair to target in degrees
//        double ta = LimelightHelpers.getTA("");  // Target area (0% to 100% of image)
//        boolean hasTarget = LimelightHelpers.getTV(""); // Do you have a valid target?
        
        // First, tell Limelight your robot's current orientation
        LimelightHelpers.SetRobotOrientation(
                "",
                180+this.robotState.getEstimatedPose().getRotation().getDegrees(),
                this.robotState.getVelocity().omegaRadiansPerSecond
        );

        // Get the pose estimate
        LimelightHelpers.PoseEstimate limelightMeasurementMT2 = LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2("");

        if (limelightMeasurementMT2.pose.equals(Pose2d.kZero)) {
            return;
        }

        this.lastVisionObservation = new PoseEstimator.VisionPoseObservation(
                limelightMeasurementMT2.timestampSeconds,
                limelightMeasurementMT2.pose,
                VisionConstants.VISION_LINEAR_STANDARD_DEVIATION_METERS,
                VisionConstants.VISION_ANGULAR_STANDARD_DEVIATION_RAD);

        // Add it to your pose estimator
        this.robotState.addVisionObservation(this.lastVisionObservation);

    }
}