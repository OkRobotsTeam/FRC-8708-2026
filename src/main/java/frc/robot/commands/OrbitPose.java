package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;

import frc.lib.commands.OrbitPoseMaintainRadiusBase;
import frc.lib.util.LoggedTuneableProfiledPID;
import frc.robot.subsystems.Drive.Drive;

/**
 * Command that orbits around a target pose while:
 *  - Maintaining initial radius
 *  - Aligning rotation to target
 *  - Allowing driver tangential control
 */
public class OrbitPose extends OrbitPoseMaintainRadiusBase {

    // Radial controller maintains constant distance
    private static final LoggedTuneableProfiledPID radialController =
            new LoggedTuneableProfiledPID(
                    "OrbitPose/RadialController",
                    3.0,  // kP
                    0.0,  // kI
                    0.1,  // kD
                    3.0,  // max velocity constraint
                    0.0   // max acceleration constraint (adjust if needed)
            );

    // Angular controller aligns robot heading to target rotation
    private static final LoggedTuneableProfiledPID angularController =
            new LoggedTuneableProfiledPID(
                    "OrbitPose/AngularController",
                    4.0,  // kP
                    0.0,  // kI
                    0.1,  // kD
                    6.0,  // max angular velocity
                    10.0  // max angular acceleration
            );

    /**
     * @param drive Drive subsystem
     * @param targetPose Pose to orbit around
     * @param tangentialInput Driver input controlling orbit direction/speed
     */
    public OrbitPose(
            Drive drive,
            Supplier<Pose2d> targetPose,
            DoubleSupplier tangentialInput)
    {
        super(
                drive,
                targetPose,
                tangentialInput,
                radialController,
                angularController
        );
    }
}
