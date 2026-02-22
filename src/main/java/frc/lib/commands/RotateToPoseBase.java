// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.commands;

import static edu.wpi.first.units.Units.Radians;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.LoggedTuneableProfiledPID;
import frc.robot.RobotState;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.Drive.Drive;

public abstract class RotateToPoseBase extends Command {
    private final RobotState robotState = RobotState.getInstance();

    private final Drive drive;
    private final Supplier<Translation2d> targetTranslation;
    private final DoubleSupplier joystickXInput;
    private final DoubleSupplier joystickYInput;
    private final LoggedTuneableProfiledPID angularController;

    public RotateToPoseBase(
        Drive drive,
        Supplier<Translation2d> targetTranslation,
        DoubleSupplier joystickXInput,
        DoubleSupplier joystickYInput,
        LoggedTuneableProfiledPID angularController)
    {
        this.drive = drive;
        this.targetTranslation = targetTranslation;
        this.joystickXInput = joystickXInput;
        this.joystickYInput = joystickYInput;
        this.angularController = angularController;

        angularController.enableContinuousInput(-Math.PI, Math.PI);
        addRequirements(drive);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize()
    {
        ChassisSpeeds fieldVelocity =
            ChassisSpeeds.fromRobotRelativeSpeeds(drive.getChassisSpeeds(),
                robotState.getEstimatedPose().getRotation());

        angularController.reset(
            robotState.getEstimatedPose().getRotation().getRadians(),
            fieldVelocity.omegaRadiansPerSecond);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute()
    {
        // Checks if tunable values for PID have changed and updates them if so
        angularController.updatePID();
        Pose2d currentPose = robotState.getEstimatedPose();
        Translation2d goalTranslation = targetTranslation.get();
        Logger.recordOutput("/RotateToPose/currentPose", currentPose);
        Logger.recordOutput("/RotateToPose/goalTranslation", goalTranslation);


        Rotation2d angleToTarget =
                goalTranslation
                        .minus(currentPose.getTranslation())
                        .getAngle();

        double angularOutput = angularController.calculate(
                currentPose.getRotation().getRadians(),
                angleToTarget.getRadians());

        Logger.recordOutput("/RotateToPose/angularOutput", angularOutput);
        Logger.recordOutput("/RotateToPose/angularController/angleToTarget", angleToTarget);
        Logger.recordOutput("/RotateToPose/angularController/currentAngle", currentPose.getRotation().getRadians());
        Logger.recordOutput("/RotateToPose/angularController/targetAngle", angleToTarget.getRadians());
        
        DriveCommands.joystickDrive(drive, joystickYInput, joystickXInput, () -> angularOutput);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished()
    {
        return false;
    }

    public Angle getAngularError()
    {
        return Radians.of(angularController.getPositionError());
    }
}
