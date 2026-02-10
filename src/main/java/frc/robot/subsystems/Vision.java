//package frc.robot.subsystems;
//
//import static frc.robot.subsystems.VisionConstants.*;
//
//import edu.wpi.first.math.VecBuilder;
//import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.lib.posestimator.PoseEstimator;
//import frc.lib.util.LimelightHelpers;
//import frc.robot.subsystems.Drive.GyroIO;
//
//public class Vision extends SubsystemBase {
//
//    PoseEstimator pose_estimator;
//    GyroIO gyro;
//
//    public Vision(PoseEstimator pose_estimator, GyroIO gyro)
//    {
//        this.pose_estimator = pose_estimator;
//        this.gyro = gyro;
//    }
//
//    @Override
//    public void periodic()
//    {
//        double tx = LimelightHelpers.getTX("");  // Horizontal offset from crosshair to target in degrees
//        double ty = LimelightHelpers.getTY("");  // Vertical offset from crosshair to target in degrees
//        double ta = LimelightHelpers.getTA("");  // Target area (0% to 100% of image)
//        boolean hasTarget = LimelightHelpers.getTV(""); // Do you have a valid target?
//
//        yaw = this.gyro.yawPosition;
//        yaw_rate = this.gyro.yawVelocityRadPerSec;
//
//        // First, tell Limelight your robot's current orientation
//        LimelightHelpers.SetRobotOrientation("", yaw, yaw_rate, 0.0, 0.0, 0.0, 0.0);
//
//        // Get the pose estimate
//        LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("");
//
//        // Add it to your pose estimator
//        pose_estimator.setVisionMeasurementStdDevs(VecBuilder.fill(.5, .5, 9999999));
//        pose_estimator.addVisionMeasurement(
//                limelightMeasurement.pose,
//                limelightMeasurement.timestampSeconds
//        );
//    }
//}