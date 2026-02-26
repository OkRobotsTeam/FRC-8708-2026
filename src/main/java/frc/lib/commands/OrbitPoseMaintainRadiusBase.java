package frc.lib.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;

import frc.lib.util.LoggedTuneableProfiledPID;
import frc.robot.RobotState;
import frc.robot.subsystems.Drive.Drive;

public abstract class OrbitPoseMaintainRadiusBase extends Command {

    private final RobotState robotState = RobotState.getInstance();

    private final Drive drive;
    private final Supplier<Pose2d> targetPose;
    private final DoubleSupplier tangentialInput;

    private final LoggedTuneableProfiledPID radialController;
    private final LoggedTuneableProfiledPID angularController;

    private double initialRadiusMeters;

    public OrbitPoseMaintainRadiusBase(
            Drive drive,
            Supplier<Pose2d> targetPose,
            DoubleSupplier tangentialInput,
            LoggedTuneableProfiledPID radialController,
            LoggedTuneableProfiledPID angularController)
    {
        this.drive = drive;
        this.targetPose = targetPose;
        this.tangentialInput = tangentialInput;
        this.radialController = radialController;
        this.angularController = angularController;

        angularController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(drive);
    }

    @Override
    public void initialize() {

        Pose2d robotPose = robotState.getEstimatedPose();
        Pose2d target = targetPose.get();

        // Capture initial radius
        initialRadiusMeters =
                robotPose.getTranslation().getDistance(target.getTranslation());

        radialController.reset(0.0);

        angularController.reset(
                robotPose.getRotation().getRadians(),
                drive.getChassisSpeeds().omegaRadiansPerSecond
        );
    }

    @Override
    public void execute() {

        radialController.updatePID();
        angularController.updatePID();

        Pose2d robotPose = robotState.getEstimatedPose();
        Pose2d target = targetPose.get();

        Translation2d robotToTarget =
                robotPose.getTranslation().minus(target.getTranslation());

        double currentRadius = robotToTarget.getNorm();

        // Avoid divide by zero
        if (currentRadius < 1e-4) return;

        Translation2d radialUnit =
                robotToTarget.div(currentRadius);

        // Tangent vector = rotate radial 90 degrees
        Translation2d tangentUnit =
                new Translation2d(
                        -radialUnit.getY(),
                        radialUnit.getX()
                );

        // PID on radius error
        double radialCorrection =
                radialController.calculate(currentRadius, initialRadiusMeters);

        Translation2d radialVelocity =
                radialUnit.times(radialCorrection);

        Translation2d tangentialVelocity =
                tangentUnit
                        .times(tangentialInput.getAsDouble())
                        .times(drive.getMaxLinearSpeedMetersPerSec());

        Translation2d totalVelocity =
                radialVelocity.plus(tangentialVelocity);

        double angularOutput =
                angularController.calculate(
                        robotPose.getRotation().getRadians(),
                        target.getRotation().getRadians()
                );

        ChassisSpeeds speeds =
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        totalVelocity.getX(),
                        totalVelocity.getY(),
                        angularOutput,
                        robotPose.getRotation()
                );

        drive.runVelocityInternal(speeds, false);
    }

    public Distance getRadiusError() {
        return Meters.of(radialController.getPositionError());
    }

    public Angle getAngularError() {
        return Radians.of(angularController.getPositionError());
    }
}
