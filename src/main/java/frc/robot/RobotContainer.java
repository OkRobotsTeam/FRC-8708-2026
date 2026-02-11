/*
 * Copyright (C) 2025 Windham Windup
 *
 * This program is free software: you can redistribute it and/or modify it under the terms of the
 * GNU General Public License as published by the Free Software Foundation, either version 3 of the
 * License, or any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without
 * even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with this program. If
 * not, see <https://www.gnu.org/licenses/>.
 */

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.commands.SteppableCommandGroup;
import frc.lib.util.LoggedDashboardChooser;
import frc.lib.util.LoggedTunableNumber;
import frc.lib.util.LoggedTuneableProfiledPID;
import frc.lib.util.PointInPolygon;
import frc.lib.util.CommandXboxControllerExtended;
import frc.lib.util.GamePieceVisualizer;
import frc.robot.Constants.PathConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.DriveToPose;
import frc.robot.commands.OnTheFlyPathCommand;
import frc.robot.commands.autos.NoneAuto;
import frc.robot.subsystems.Drive.Drive;
import frc.robot.subsystems.Drive.DriveConstants;
import frc.robot.subsystems.Vision.Vision;
import frc.robot.util.BallSimulator;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.FeetPerSecond;

import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Transfer;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.littletonrobotics.junction.Logger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
@SuppressWarnings("unused")
public class RobotContainer {
    private final RobotState robotState = RobotState.getInstance();

    // Subsystems
    public final Drive drive;

    Shooter shooter = new Shooter();
    Transfer transfer = new Transfer();

    private final Vision vision = new Vision(this.robotState);
    // Controller
    private final CommandXboxControllerExtended controller = new CommandXboxControllerExtended(0);

    // Dashboard inputs

    private final LoggedDashboardChooser<PathPlannerAuto> autoChooser;
    private final LoggedDashboardChooser<Boolean> conditionalChooser;
    public static Field2d autoPreviewField = new Field2d();

    private final Trigger inAllianceRegionTrigger;

    /**
     * The container for the robot. Contains subsystems, IO devices, and commands.
     */
    public RobotContainer() {
        RobotConfig config = null;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
        }
        drive = DriveConstants.get();

        AutoBuilder.configure(
                RobotState.getInstance()::getEstimatedPose, // Robot pose supplier
                RobotState.getInstance()::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
                RobotState.getInstance()::getVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                drive::pathPlannerDrive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
                ), config, () -> {
                    var alliance = DriverStation.getAlliance();
                    return alliance.filter(value -> value == DriverStation.Alliance.Red).isPresent();
                },
                drive // Reference to this subsystem to set requirements
        );


        conditionalChooser = new LoggedDashboardChooser<>("Conditional Choice");
        conditionalChooser.addOption("True", true);
        conditionalChooser.addOption("False", false);

        // Set up auto routines
        autoChooser = new LoggedDashboardChooser<PathPlannerAuto>("Auto Choices");

        for (String autoName : (AutoBuilder.getAllAutoNames())) {
            PathPlannerAuto autoCommand = (PathPlannerAuto) AutoBuilder.buildAuto(autoName);
            autoChooser.addOption(autoName, autoCommand);
        }
        SmartDashboard.putData("Auto Preview", autoPreviewField);

        autoChooser.addDefaultOption("None", new NoneAuto());

        autoChooser.addOption("test", new NoneAuto());

        autoChooser.onChange(auto -> {
            try {
                List<PathPlannerPath> path = PathPlannerAuto.getPathGroupFromAutoFile(auto.getName());
                ArrayList<Pose2d> poses = new ArrayList<>();
                for (PathPlannerPath pathPart : path) {
                    poses.addAll(pathPart.getPathPoses());
                }
                System.out.println("Previewing path: " + auto.getName() + " Num Poses:" + poses.size());
                autoPreviewField.getObject("path").setPoses(poses);
            } catch (Exception e) {
                System.out.println("Couldn't get path group for auto: " + auto.getName());
                autoPreviewField.getObject("path").setPoses();

                //throw new RuntimeException(e);
            }

// Display the path on the dashboard field widget
            //autoPreviewField.getObject("path").setPoses(auto.());
        });

        inAllianceRegionTrigger = new Trigger(() -> PointInPolygon.pointInPolygon(
                robotState.getEstimatedPose().getTranslation(),
                FieldConstants.ALLIANCE_STATION_POLYGON));

        // Configure the button bindings
        configureButtonBindings();

        GamePieceVisualizer algae = new GamePieceVisualizer("Algae",
                new Pose3d(new Translation3d(3, 3, 1), new Rotation3d(0, 0, 0)));


        // Detect if controllers are missing / Stop multiple warnings
        DriverStation.silenceJoystickConnectionWarning(true);
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
     * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        // Default command, normal field-relative drive
        drive.setDefaultCommand(
                DriveCommands.joystickDrive(
                        drive,
                        () -> -controller.getLeftY(),
                        () -> -controller.getLeftX(),
                        () -> -controller.getRightX()));

//
//            controller.b().whileTrue(Commands.runOnce(() -> drive.getModule(0).test()));
//            controller.b().whileTrue(Commands.runOnce(() -> drive.getModule(1).test()));
//            controller.b().whileTrue(Commands.runOnce(() -> drive.getModule(2).test()));
//            controller.b().whileTrue(Commands.runOnce(() -> drive.getModule(3).test()));

        controller.rightBumper().onTrue(Commands.runOnce((shooter::topFaster), shooter));
        controller.leftBumper().onTrue(Commands.runOnce((shooter::topSlower), shooter));
        controller.rightTrigger().onTrue(Commands.runOnce((shooter::bottomFaster), shooter));
        controller.leftTrigger().onTrue(Commands.runOnce((shooter::bottomSlower), shooter));
        //controller.x().onTrue(Commands.runOnce(topShooter::stop,topShooter).andThen(Commands.runOnce(bottomShooter::stop,bottomShooter)));
        controller.y().onTrue(Commands.runOnce(shooter::toggleIsRunning, shooter));
        controller.a().onTrue(Commands.runOnce(() -> transfer.setBothPercent(0.5)));
        controller.a().onFalse(Commands.runOnce(() -> transfer.setBothPercent(0.0)));
        controller.b().onTrue(Commands.runOnce(() -> transfer.setBothPercent(-0.5)));
        controller.b().onFalse(Commands.runOnce(() -> transfer.setBothPercent(0.0)));


//        controller.b().whileTrue(
//                Commands.runOnce(drive::test)
//        );
        // Lock to 0° when A button is held
        controller
                .a()
                .whileTrue(
                        DriveCommands.joystickDriveAtAngle(
                                drive,
                                () -> -controller.getLeftY(),
                                () -> -controller.getLeftX(),
                                () -> new Rotation2d()));

        // Switch to X pattern when X button is pressed
        // controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

        // Reset gyro to 0° when B button is pressed
        controller
                .b()
                .onTrue(
                        Commands.runOnce(
                                        () -> robotState.resetPose(
                                                new Pose2d(robotState.getEstimatedPose().getTranslation(),
                                                        new Rotation2d())))
                                .ignoringDisable(true));

        // Pathfind to Pose when the Y button is pressed
        controller.y().onTrue(
                DriveCommands.pathFindToPose(() -> robotState.getEstimatedPose(),
                        new Pose2d(1, 4, Rotation2d.kZero),
                        PathConstants.ON_THE_FLY_PATH_CONSTRAINTS, MetersPerSecond.of(0.0),
                        PathConstants.PATHGENERATION_DRIVE_TOLERANCE));

        // On-the-fly path with waypoints while the Right Bumper is held
        controller.rightBumper().whileTrue(
                new OnTheFlyPathCommand(drive, () -> robotState.getEstimatedPose(),
                        new ArrayList<>(Arrays.asList()), // List
                        // of
                        // waypoints
                        new Pose2d(6, 6, Rotation2d.k180deg), PathConstants.ON_THE_FLY_PATH_CONSTRAINTS,
                        MetersPerSecond.of(0.0), false, PathConstants.PATHGENERATION_DRIVE_TOLERANCE,
                        PathConstants.PATHGENERATION_ROT_TOLERANCE));


        LoggedTunableNumber ballVel = new LoggedTunableNumber("Ball Sim Velocity (fps)", 15);
        SmartDashboard.putData("Shoot Ball", Commands
                .runOnce(() -> BallSimulator.launch(FeetPerSecond.of(ballVel.getAsDouble()))));

        GamePieceVisualizer algaeViz =
                new GamePieceVisualizer("Algae #1", new Pose3d(1, 1, 1, new Rotation3d()));
        SmartDashboard.putData("Hide Algae", Commands.runOnce(() -> algaeViz.hide()));

        LoggedTuneableProfiledPID linearController =
                new LoggedTuneableProfiledPID("DriveToPose/LinearController", 3.0, 0, 0.1, 0, 3.0);

        SmartDashboard.putData("DriveToPose Command",
                new DriveToPose(drive, () -> new Pose2d(5, 5, Rotation2d.fromDegrees(90)))
                        .withTolerance(Inches.of(3), Degrees.of(5)));

        Command steppableCommand = new SteppableCommandGroup(
                controller.x(),
                controller.y(),
                Commands.runOnce(() -> System.out.println("Step 1")),
                Commands.runOnce(() -> System.out.println("Step 2")),
                Commands.runOnce(() -> System.out.println("Step 3")));

        SmartDashboard.putData("Steppable Command", steppableCommand);

        // controller.x()
        // .whileTrue(new DriveToPose(drive, () -> new Pose2d(5, 5, Rotation2d.fromDegrees(90)))
        // .withTolerance(Inches.of(3), Degrees.of(5)));

        // controller.x()
        // .whileTrue(new AlignToPose(drive, () -> new Pose2d(5, 5, Rotation2d.fromDegrees(0)),
        // AlignMode.STRAFE, () -> controller.getRightX()));

        // Right bumper: Shoot on the Move

        inAllianceRegionTrigger.onTrue(
                Commands.runOnce(() -> Logger.recordOutput("InAllianceRegionTrigger", true))
                        .ignoringDisable(true));
        inAllianceRegionTrigger.onFalse(
                Commands.runOnce(() -> Logger.recordOutput("InAllianceRegionTrigger", false))
                        .ignoringDisable(true));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }

    /**
     * This function is called periodically by Robot.java when disabled.
     */
    public void checkStartPose() {

        /* Starting pose checker for auto */
        autoPreviewField.setRobotPose(robotState.getEstimatedPose());

        try {
            double distanceFromStartPose = robotState.getEstimatedPose().getTranslation()
                    .getDistance(autoPreviewField.getObject("path").getPoses().get(0).getTranslation());
            double degreesFromStartPose = Math.abs(robotState.getEstimatedPose().getRotation()
                    .minus(
                            autoPreviewField.getObject("path").getPoses().get(0).getRotation())
                    .getDegrees());

            SmartDashboard.putNumber("Auto Pose Check/Inches from Start",
                    Math.round(distanceFromStartPose * 100.0) / 100.0);
            SmartDashboard.putBoolean(
                    "Auto Pose Check/Robot Position within "
                            + PathConstants.STARTING_POSE_DRIVE_TOLERANCE.in(Inches) + " inches",
                    distanceFromStartPose < PathConstants.STARTING_POSE_DRIVE_TOLERANCE.in(Inches));
            SmartDashboard.putNumber("Auto Pose Check/Degrees from Start",
                    Math.round(degreesFromStartPose * 100.0) / 100.0);
            SmartDashboard.putBoolean(
                    "Auto Pose Check/Robot Rotation within "
                            + PathConstants.STARTING_POSE_ROT_TOLERANCE_DEGREES + " degrees",
                    degreesFromStartPose < PathConstants.STARTING_POSE_ROT_TOLERANCE_DEGREES
                            .in(Degrees));

        } catch (Exception e) {
            SmartDashboard.putNumber("Auto Pose Check/Inches from Start", -1);
            SmartDashboard.putBoolean(
                    "Auto Pose Check/Robot Position within "
                            + PathConstants.STARTING_POSE_DRIVE_TOLERANCE.in(Inches) + " inches",
                    false);
            SmartDashboard.putNumber("Auto Pose Check/Degrees from Start", -1);
            SmartDashboard.putBoolean(
                    "Auto Pose Check/Robot Rotation within "
                            + PathConstants.STARTING_POSE_ROT_TOLERANCE_DEGREES.in(Degrees) + " degrees",
                    false);
        }
    }

    public void init() {
    }

    public void teleopInit() {
    }

    public void autonomousInit() {
    }

    public void testInit() {
    }

    public void periodic() {
    }

    public void teleopPeriodic() {
    }

    public void testPeriodic() {
    }
}
