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

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import frc.lib.io.servo.ServoIO;

import java.util.List;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
    public static final Mode simMode = Mode.SIM;
    public static Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;


    public static final CurrentLimitsConfigs FLYWHEEL_CURRENT_LIMITS =
            new CurrentLimitsConfigs()
                    .withSupplyCurrentLimit(30)
                    .withSupplyCurrentLimitEnable(true);

    public static final CurrentLimitsConfigs INTAKE_WHEEL_CURRENT_LIMITS =
            new CurrentLimitsConfigs()
                    .withSupplyCurrentLimit(30)
                    .withSupplyCurrentLimitEnable(true);


    public static final CurrentLimitsConfigs INTAKE_EXTENDER_CURRENT_LIMITS =
            new CurrentLimitsConfigs()
                    .withSupplyCurrentLimit(30)
                    .withSupplyCurrentLimitEnable(true);


    public static final CurrentLimitsConfigs INJECTOR_CURRENT_LIMITS =
            new CurrentLimitsConfigs()
                    .withSupplyCurrentLimit(30)
                    .withSupplyCurrentLimitEnable(true);


    public static final CurrentLimitsConfigs TRANSFER_CURRENT_LIMITS =
            new CurrentLimitsConfigs()
                    .withSupplyCurrentLimit(60) //TODO Update once changed
                    .withSupplyCurrentLimitEnable(true);


    public static final CurrentLimitsConfigs HOOD_CURRENT_LIMITS =
            new CurrentLimitsConfigs()
                    .withSupplyCurrentLimit(30)
                    .withSupplyCurrentLimitEnable(true);


    public static final boolean tuningMode = true;

    public static enum Mode {
        ALPHA,
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
    }

    public static class RobotConstants {
        public static String serial;
        public static boolean isComp;
        public static boolean isAlpha;

        // TODO: Fill in with real serial number prefixes. Figure out by displaying/logging String
        // serial.
        public static final String compSerial = "0001";
        public static final String alphaSerial = "031df993";

        static {
            if (Robot.isReal()) {
                // Roborio id recognition
                serial = System.getenv("serialnum");
            } else {
                serial = "8708";
            }
            RobotConstants.isComp = serial.startsWith(RobotConstants.compSerial);
            RobotConstants.isAlpha = serial.startsWith(RobotConstants.alphaSerial);
        }
    }

    public static RobotType robotType = RobotConstants.isComp ? RobotType.COMP
        : RobotConstants.isAlpha ? RobotType.ALPHA : RobotType.NONE;

    public enum RobotType {
        COMP,
        ALPHA,
        NONE
    }

    public static final class ShooterConstants {
        public static final int MOTOR_1_ID = 1;
        public static final int MOTOR_2_ID = 2;
        public static final int HOOD_ID = 3;
        public static final int INJECTOR_ID = 4;
        public static final int TRANSFER_ID = 5;
        public static final int SERVO_1 = 0;
        public static final int SERVO_2 = 1;
        public static final double HOOD_STARTING_POSITION = 0.0; //TODO UPDATE
        public static final int TICKS_PER_REV = 2048; // TalonFX integrated sensor
        public static final double FLYWHEEL_P = 0.2;
        public static final double FLYWHEEL_I = 0;
        public static final double FLYWHEEL_D = 0.01;
        public static final double FLYWHEEL_V = 0.15;
        public static final double HOOD_P = 0.2;
        public static final double HOOD_I = 0;
        public static final double HOOD_D = 0.01;
        public static final double HOOD_V = 0.125;
        public static final double TRANSFER_P = 0.1;
        public static final double TRANSFER_D = 0.01;
        public static final double TRANSFER_V = 0.125;
        public static final int MAXIMUM_RPM = 6000;
        public static final int ENCODER_CHANNEL_A = 1;
        public static final int ENCODER_CHANNEL_B = 2;
        public static final boolean ENCODER_REVERSED = false;
        public static final CounterBase.EncodingType ENCODER_ENCODING_TYPE = Encoder.EncodingType.k4X;
        public static final int MAXIMUM_ANGULAR_ROTATIONS = 10; //TODO UPDATE
        public static final double SPEED_TOLERANCE = 5; //TODO UPDATE
        public static final double SPEED_WHEN_OUTSIDE_ZONE = 100;
    }


    public static final class IntakeConstants {
        public static final int INTAKE_MOTOR_ID = 6;
        public static final int INTAKE_EXTENDER_ID = 7;
        public static final int TICKS_PER_REV = 2048; // TalonFX integrated sensor
        public static final double KP = 0.1;
        public static final double KI = 0;
        public static final double KD = 0;
        public static final double KV = 0.125;
        public static final int EXTENDED_POSITION = 5; //TODO GET POSITION
        public static final int RETRACTED_POSITION = 0; //TODO GET POSITION
        public static final double INTAKE_ROTATION_GEAR_RATIO = 19.42;
        public static final double WIGGLE_ROTATIONS = 1;
    }

    public static final class PathConstants {

        public static final Distance STARTING_POSE_DRIVE_TOLERANCE =
            Inches.of(3.0); // For auto
        public static final Angle STARTING_POSE_ROT_TOLERANCE_DEGREES = Degrees.of(5.0);

        public static final Distance PATHGENERATION_DRIVE_TOLERANCE = Inches.of(3.0);
        public static final Angle PATHGENERATION_ROT_TOLERANCE = Degrees.of(5.0);
        // Tune the maxAcceleration, maxAngularVelocityRadPerSec, and
        // maxAngularAccelerationRacPerSecSq constraints for pathfinding

        // Pathing constants for teleop
        public static final List<Pose2d> AUTO_ALIGN_TARGET_POSES = List.of(
                new Pose2d(13.875, 4.145, Rotation2d.fromDegrees(0)),
                new Pose2d(14.5, 7, Rotation2d.fromDegrees(45))
                // TODO: Add more target poses here
        );

    }
}
