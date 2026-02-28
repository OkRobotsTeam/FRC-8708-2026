// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.lib.commands.RotateToCardinalDirectionBase;
import frc.lib.util.LoggedTuneablePID;
import frc.robot.subsystems.Drive.Drive;

import java.util.function.DoubleSupplier;

public class RotateToCardinalDirection extends RotateToCardinalDirectionBase {
    private final static LoggedTuneablePID angularController =
            new LoggedTuneablePID("RotateToCardinalDirection/AngularController", 0.4, 0.001, 0.03);


    public RotateToCardinalDirection(Drive drive,
                                     Rotation2d angleOffset,
                                     DoubleSupplier joystickXInput,
                                     DoubleSupplier joystickYInput
                        )
    {
        super(
            drive,
            angleOffset,
            joystickXInput,
            joystickYInput,
            angularController
        );
    }
}
