// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.LoggedTuneablePID;
import frc.robot.RobotState;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.Drive.Drive;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static edu.wpi.first.math.MathUtil.clamp;
import static edu.wpi.first.units.Units.Radians;

public abstract class RotateToCardinalDirectionBase extends Command {
    private final RobotState robotState = RobotState.getInstance();

    private final Drive drive;
    private final Rotation2d angleOffset;
    private final DoubleSupplier joystickXInput;
    private final DoubleSupplier joystickYInput;
    private final LoggedTuneablePID angularController;
    private Rotation2d targetRotation;

    public RotateToCardinalDirectionBase(
        Drive drive,
        Rotation2d angleOffset,
        DoubleSupplier joystickXInput,
        DoubleSupplier joystickYInput,
        LoggedTuneablePID angularController)
    {
        this.drive = drive;
        this.angleOffset = angleOffset;
        this.joystickXInput = joystickXInput;
        this.joystickYInput = joystickYInput;
        this.angularController = angularController;
        this.angularController.setTolerance(Math.toRadians(3));

        angularController.enableContinuousInput(-Math.PI, Math.PI);
        addRequirements(drive);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize()
    {
        angularController.reset();

        Pose2d currentPose = robotState.getEstimatedPose();
        Logger.recordOutput("/RotateToCardinalDirection/currentPose", currentPose);

        Rotation2d[] optionAngles = new Rotation2d[]
                {Rotation2d.fromDegrees(0),
                        Rotation2d.fromDegrees(90),
                        Rotation2d.fromDegrees(180),
                        Rotation2d.fromDegrees(270)};

        Double[] bestError = new Double[optionAngles.length];

        for (int i = 0; i < optionAngles.length; i++) {
            bestError[i] = Math.abs(getBestError(currentPose.getRotation(), optionAngles[i].plus(angleOffset)));
        }

        int lowestErrorIndex = 0;
        for (int i = 1; i < bestError.length; i++) {
            if (bestError[i] < bestError[lowestErrorIndex]) {
                lowestErrorIndex = i;
            }
        }

        targetRotation = optionAngles[lowestErrorIndex].plus(angleOffset);

    }

    private double getBestError(Rotation2d currentRotation, Rotation2d targetRotation)
    {
        return MathUtil.inputModulus((targetRotation.minus(currentRotation)).getDegrees(), -180, 180);
    }

    @Override
    public void execute()
    {
        // Checks if tunable values for PID have changed and updates them if so
        angularController.updatePID();
        Pose2d currentPose = robotState.getEstimatedPose();
        Logger.recordOutput("/RotateToCardinalDirection/currentPose", currentPose);

        double angularOutput = clamp(
                angularController.calculate(
                        currentPose.getRotation().getRadians(),
                        targetRotation.getRadians()),
                -1, 1);

        Logger.recordOutput("/RotateToCardinalDirection/angularController/PIDOutput", angularOutput);
        Logger.recordOutput("/RotateToCardinalDirection/angularController/PIDError", Math.toDegrees(angularController.getError()));
        Logger.recordOutput("/RotateToCardinalDirection/angularController/targetRotation", targetRotation);
        Logger.recordOutput("/RotateToCardinalDirection/angularController/currentAngle", currentPose.getRotation().getRadians());

        DriveCommands.joystickDrive(drive, joystickYInput, joystickXInput, () -> angularOutput, false);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished()
    {
        return false;
    }

    public Angle getAngularError()
    {
        return Radians.of(angularController.getError());
    }
}
