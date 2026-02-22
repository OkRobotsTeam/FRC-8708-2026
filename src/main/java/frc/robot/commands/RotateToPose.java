// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.lib.commands.RotateToPoseBase;
import frc.lib.util.LoggedTuneableProfiledPID;
import frc.robot.subsystems.Drive.Drive;

public class RotateToPose extends RotateToPoseBase {

    private final static LoggedTuneableProfiledPID angularController =
        new LoggedTuneableProfiledPID("DriveToPose/AngularController", 3.0, 0, 0, 0, 0);

    public RotateToPose(Drive drive,
                        Supplier<Translation2d> targetPose,
                        DoubleSupplier joystickXInput,
                        DoubleSupplier joystickYInput
                        )
    {
        super(
            drive,
            targetPose,
            joystickXInput,
            joystickYInput,
            angularController
        );
    }
}
