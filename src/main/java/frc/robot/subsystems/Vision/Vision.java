package frc.robot.subsystems.Vision;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.posestimator.PoseEstimator;
import frc.lib.util.LimelightHelpers;
import frc.robot.RobotState;

public class Vision extends SubsystemBase {

    private final RobotState robotState;


    public Vision(RobotState robotState) {
        this.robotState = robotState;
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
                this.robotState.getRotation().getDegrees(),
                this.robotState.getVelocity().omegaRadiansPerSecond
        );

        // Get the pose estimate
        LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("");

        // Add it to your pose estimator
        this.robotState.addVisionObservation(
                new PoseEstimator.VisionPoseObservation(
                        limelightMeasurement.timestampSeconds,
                        limelightMeasurement.pose,
                        VisionConstants.VISION_LINEAR_STANDARD_DEVIATION_METERS,
                        VisionConstants.VISION_ANGULAR_STANDARD_DEVIATION_RAD_PER_SECOND)
        );
    }
}