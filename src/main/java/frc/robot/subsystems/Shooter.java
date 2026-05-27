package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

/**
 * Subsystem to control two TalonFX motors independently (or optionally make the
 * top motor follow the bottom).
 */
public class Shooter extends SubsystemBase {
//    public final LoggedTuneablePID rotationPID = new LoggedTuneablePID("/Shooter/RotationPID", ShooterConstants.ANGLER_P, ShooterConstants.ANGLER_I, ShooterConstants.ANGLER_D);
//    public final LoggedTuneablePID shooterPID = new LoggedTuneablePID("/Shooter/ShooterPID", ShooterConstants.FLYWHEEL_P, ShooterConstants.FLYWHEEL_I, ShooterConstants.FLYWHEEL_D);

    public final TalonFX flywheelMotor1;
    public final TalonFX flywheelMotor2;
    private final TalonFX injector;
    private final TalonFX transfer;
    private final Servo hoodServo1 = new Servo(ShooterConstants.SERVO_1);
    private final Servo hoodServo2 = new Servo(ShooterConstants.SERVO_2);

    private final PWMSparkMax servo1 = new PWMSparkMax(2);
    AnalogPotentiometer potentiometer = new AnalogPotentiometer(0, 1, 0);


    private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0);

    boolean isShooting = false;
    public double motorSpeed = 0;
    public double manualSpeed = 45;
    private double automaticSpeed = 0.0;

    private final RobotState robotState = RobotState.getInstance();

    public double automaticHoodPosition = 0.0;
    public double hoodPosition = 0;
    public double manualHoodPosition = 0;



    public static ArrayList<Double> shooterSpeeds = new ArrayList<Double>(List.of(49.0, 45.0, 52.0, 49.0, 0.0, 55.0, 100.0)); // with hood
    //    public ArrayList<Double> shooterSpeeds = new ArrayList<Double>(List.of(49.0, 55.0, 70.0)); //no hood
    public static ArrayList<Double> hoodPositions = new ArrayList<Double>(List.of(0.3, 0.15, 0.16, 0.3, 0.0, 0.6, 0.85)); // five wire servo
    //    public ArrayList<Double> hoodPositions = new ArrayList<Double>(List.of(0.35, 0.32, 0.33, 0.7)); // three wire servo
    //    public ArrayList<Double> hoodPositions = new ArrayList<Double>(List.of(0.0, 0.0, 0.0)); // no hood
    public int currentPreset = 0;

    private static final InterpolatingDoubleTreeMap flywheelSpeedMap = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap hoodAngleMap = new InterpolatingDoubleTreeMap();

    static {
//        for (int i = 1; i < 4; i++) {
//            flywheelSpeedMap.put(Constants.PathConstants.AUTO_ALIGN_TARGET_POSES.get(i-1).getTranslation()
//                    .getDistance(FieldConstants.RED_GOAL_POSITION), shooterSpeeds.get(i));
//
//            hoodAngleMap.put(Constants.PathConstants.AUTO_ALIGN_TARGET_POSES.get(i-1).getTranslation()
//                    .getDistance(FieldConstants.RED_GOAL_POSITION), hoodPositions.get(i));
//
//        }
        flywheelSpeedMap.put(ShooterConstants.CLOSEST_DISTANCE, ShooterConstants.CLOSEST_SPEED);
        flywheelSpeedMap.put(ShooterConstants.INTERPOLATE_DISTANCE_1, ShooterConstants.INTERPOLATE_SPEED_1);
        flywheelSpeedMap.put(ShooterConstants.INTERPOLATE_DISTANCE_2, ShooterConstants.INTERPOLATE_SPEED_2);
        flywheelSpeedMap.put(ShooterConstants.INTERPOLATE_DISTANCE_3, ShooterConstants.INTERPOLATE_SPEED_3);
        flywheelSpeedMap.put(ShooterConstants.INTERPOLATE_DISTANCE_4, ShooterConstants.INTERPOLATE_SPEED_4);
        flywheelSpeedMap.put(ShooterConstants.FARTHEST_DISTANCE, ShooterConstants.FARTHEST_SPEED);

        hoodAngleMap.put(ShooterConstants.CLOSEST_DISTANCE, ShooterConstants.CLOSEST_HOOD_ANGLE);
        hoodAngleMap.put(ShooterConstants.INTERPOLATE_DISTANCE_1, ShooterConstants.INTERPOLATE_HOOD_ANGLE_1);
        hoodAngleMap.put(ShooterConstants.INTERPOLATE_DISTANCE_2, ShooterConstants.INTERPOLATE_HOOD_ANGLE_2);
        hoodAngleMap.put(ShooterConstants.INTERPOLATE_DISTANCE_3, ShooterConstants.INTERPOLATE_HOOD_ANGLE_3);
        hoodAngleMap.put(ShooterConstants.INTERPOLATE_DISTANCE_4, ShooterConstants.INTERPOLATE_HOOD_ANGLE_4);
        hoodAngleMap.put(ShooterConstants.FARTHEST_DISTANCE, ShooterConstants.FARTHEST_HOOD_ANGLE);
    }


    public Shooter() {
        flywheelMotor1 = new TalonFX(ShooterConstants.MOTOR_1_ID);
        flywheelMotor2 = new TalonFX(ShooterConstants.MOTOR_2_ID);
        injector = new TalonFX(ShooterConstants.INJECTOR_ID);
        transfer = new TalonFX(ShooterConstants.TRANSFER_ID);

        var configs = new MotorOutputConfigs();

        configs.Inverted = InvertedValue.CounterClockwise_Positive;
        configs.NeutralMode = NeutralModeValue.Brake;

        // APPLY CURRENT LIMITS
        flywheelMotor1.getConfigurator().apply(Constants.FLYWHEEL_CURRENT_LIMITS);
        flywheelMotor2.getConfigurator().apply(Constants.FLYWHEEL_CURRENT_LIMITS);
        injector.getConfigurator().apply(Constants.INJECTOR_CURRENT_LIMITS);
        transfer.getConfigurator().apply(Constants.TRANSFER_CURRENT_LIMITS);

        configs.Inverted = InvertedValue.Clockwise_Positive;
        transfer.getConfigurator().apply(configs);
        flywheelMotor2.getConfigurator().apply(configs);

        configs.Inverted = InvertedValue.CounterClockwise_Positive;
        injector.getConfigurator().apply(configs);
        flywheelMotor1.getConfigurator().apply(configs);
//        flywheelMotor2.setControl(new Follower(flywheelMotor1.getDeviceID(), MotorAlignmentValue.Opposed));

        flywheelMotor1.setPosition(0.0);
        flywheelMotor2.setPosition(0.0);
        hoodServo1.setPosition(ShooterConstants.HOOD_STARTING_POSITION);
        hoodServo2.setPosition(ShooterConstants.HOOD_STARTING_POSITION);

        TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration(); // Start with factory defaults
        Slot0Configs slot0Configs = new Slot0Configs();
        slot0Configs.kP = ShooterConstants.FLYWHEEL_P;
        slot0Configs.kV = ShooterConstants.FLYWHEEL_V;
        slot0Configs.kD = ShooterConstants.FLYWHEEL_D;

        flywheelMotor1.getConfigurator().apply(slot0Configs);
        flywheelMotor2.getConfigurator().apply(slot0Configs);

    }

    public void setShooterModeStopped() {
        isShooting = false;
        System.out.println("setShooterModeStopped");
    }

    public void setShooterModeShooting() {
        isShooting = true;
        System.out.println("setShooterModeShooting");
    }

    public void setInjectorMotor(double speed) {
        injector.setControl(dutyCycleOut.withOutput(speed));
    }

    public void setTransferMotor(double speed) {
        transfer.setControl(dutyCycleOut.withOutput(speed));
    }

    public void setManualSpeed(double input) {
        manualSpeed = MathUtil.clamp(input, 0, 100);
        updateFlywheelSpeed();
    }

    public double clampHoodPosition(double input) {
        return MathUtil.clamp(input, 0.0, 1);
    }

    public void setHoodPosition(double input) {
        manualHoodPosition = clampHoodPosition(input);
//        System.out.println("Changing manual hood angle to " + manualHoodPosition);
        updateHoodAngle();
    }

    public void faster() {
        shooterSpeeds.set(currentPreset, MathUtil.clamp((shooterSpeeds.get(currentPreset) + 1), 0, 100));
        setManualSpeed(shooterSpeeds.get(currentPreset));
        System.out.println("Faster");
    }

    public void slower() {
        shooterSpeeds.set(currentPreset, MathUtil.clamp((shooterSpeeds.get(currentPreset) - 1), 0, 100));
        setManualSpeed(shooterSpeeds.get(currentPreset));
        System.out.println("Slower");
    }


    public void angleUp() {
        hoodPositions.set(currentPreset, clampHoodPosition(hoodPositions.get(currentPreset) + 0.01));
    }

    public void angleDown() {
        hoodPositions.set(currentPreset, clampHoodPosition(hoodPositions.get(currentPreset) - 0.01));
    }

    public void selectPreset(int preset) {
        currentPreset = preset;
        setManualSpeed(shooterSpeeds.get(currentPreset));
    }

    public void nextPreset() {
        currentPreset = MathUtil.clamp(currentPreset + 1, 1, shooterSpeeds.size() - 1);
        setManualSpeed(shooterSpeeds.get(currentPreset));
        System.out.println("Next Preset");
    }

    public void previousPreset() {
        currentPreset = MathUtil.clamp(currentPreset - 1, 1, shooterSpeeds.size() - 1);
        setManualSpeed(shooterSpeeds.get(currentPreset));
        System.out.println("Previous Preset");
    }

    public void updateHoodAngle() {
        hoodPosition = manualHoodPosition;

        hoodPosition = MathUtil.clamp(hoodPosition, 0, 1);

        hoodServo1.setPosition(hoodPosition);
        hoodServo2.setPosition(hoodPosition);
        Logger.recordOutput("Shooter/LeftServo", hoodPosition);
        Logger.recordOutput("Shooter/RightServo", hoodPosition);

    }

    public void runServo() {

        double currentPosition = potentiometer.get();
        servo1.set(-(hoodPosition - currentPosition) * 10);
        Logger.recordOutput("Shooter/ServoAnalogPosition", currentPosition);
//        Logger.recordOutput("Shooter/ServoTarget", currentPosition);

    }

    public double calculateAutomaticHoodAngle() {
        Pose2d currentPose = robotState.getEstimatedPose();
        Translation2d target;
        double distance;
        if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
            target = FieldConstants.BLUE_GOAL_POSITION;
        } else {
            target = FieldConstants.RED_GOAL_POSITION;
        }
        distance = target.getDistance(currentPose.getTranslation());

        automaticHoodPosition = hoodAngleMap.get(distance);
        setHoodPosition(automaticHoodPosition);

        Logger.recordOutput("Shooter/Automatic Hood Angle",  automaticHoodPosition);

//        automaticHoodPosition = MathUtil.clamp(distance * 20, 0, 100);
        return automaticHoodPosition;
    }

    public void updateFlywheelSpeed() {
        if (isShooting) {
            setFlywheelSpeedPercent(manualSpeed);
        } else {
            setFlywheelSpeedPercent(0.0);
        }
    }

    /**
     * Set both motors to the same percent output.
     */
    public void setFlywheelSpeedPercent(double percent) {
        Logger.recordOutput("Shooter/FlywheelSpeed", percent);
//        System.out.println(percent);
        if (percent > 1.0) {
//            flywheelMotor1.setControl(new VelocityVoltage(percent));
            flywheelMotor1.setControl(new VelocityTorqueCurrentFOC(percent));
            flywheelMotor2.setControl(new VelocityTorqueCurrentFOC(percent));
        } else {
            flywheelMotor1.setControl(new DutyCycleOut(0.0));
            flywheelMotor2.setControl(new DutyCycleOut(0.0));
        }
        motorSpeed = percent;
//        System.out.println("setting both percent to " + percent);
    }

    public double calculateAutomaticFlywheelSpeed() {
        Pose2d currentPose = robotState.getEstimatedPose();
        Translation2d target;
        if (DriverStation.getAlliance().isEmpty() || DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
            target = FieldConstants.BLUE_GOAL_POSITION;
        } else {
            target = FieldConstants.RED_GOAL_POSITION;
        }
        double distance = target.getDistance(currentPose.getTranslation());

//        automaticSpeed = MathUtil.clamp(distance * 20, 0, 100);

        automaticSpeed = flywheelSpeedMap.get(distance);

        Logger.recordOutput("Shooter/AutomaticSpeed", automaticSpeed);
        Logger.recordOutput("Shooter/Distance", distance);

        return automaticSpeed;
    }

    @AutoLogOutput(key = "Shooter/CalculatedShootingPosition")
    public Translation2d calculateShootingPosition() {
        Pose2d currentPose = robotState.getEstimatedPose();
        if (DriverStation.getAlliance().equals(Optional.of(DriverStation.Alliance.Blue))) {
            if (inBlueAllianceZone(currentPose)) {
                return FieldConstants.BLUE_GOAL_POSITION;

            }
        } else {
            if (inRedAllianceZone(currentPose)) {
                return FieldConstants.RED_GOAL_POSITION;
            }
        }
        return null;
    }

    public boolean inRedAllianceZone(Pose2d currentPose) {
        return (currentPose.getMeasureX().compareTo(FieldConstants.RED_GOAL_POSITION.getMeasureX()) > 0);
    }

    public boolean inBlueAllianceZone(Pose2d currentPose) {
        return (currentPose.getMeasureX().compareTo(FieldConstants.BLUE_GOAL_POSITION.getMeasureX()) < 0);
    }

    public boolean inBlueLeftHalf(Pose2d currentPose) {
        return (currentPose.getMeasureY().compareTo(FieldConstants.BLUE_GOAL_POSITION.getMeasureY()) > 0);
    }

    public boolean inRedLeftHalf(Pose2d currentPose) {
        return (currentPose.getMeasureY().compareTo(FieldConstants.RED_GOAL_POSITION.getMeasureY()) > 0);
    }


    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        Translation2d shootingPosition = calculateShootingPosition();
//        Logger.recordOutput("Shooter/shootingPosition", shootingPosition);
        Logger.recordOutput("Shooter/ActualFlywheelMotor1Speed", flywheelMotor1.getVelocity().getValueAsDouble());
        Logger.recordOutput("Shooter/ActualFlywheelMotor2Speed", flywheelMotor2.getVelocity().getValueAsDouble());

        Logger.recordOutput("Shooter/Flywheel Motor Difference", flywheelMotor1.getVelocity().getValueAsDouble() - flywheelMotor2.getVelocity().getValueAsDouble());
        Logger.recordOutput("Shooter/Injector Motor Speed", injector.getVelocity().getValueAsDouble());
        Logger.recordOutput("Shooter/Floor Speed", transfer.getVelocity().getValueAsDouble());

        // Update the automatic presets with a new automatic speed based on our distance from the goal
        shooterSpeeds.set(Constants.ShooterConstants.AUTOMATIC_PRESET_INDEX, calculateAutomaticFlywheelSpeed());
        hoodPositions.set(Constants.ShooterConstants.AUTOMATIC_PRESET_INDEX, calculateAutomaticHoodAngle());


        if (isShooting) {
            setHoodPosition(hoodPositions.get(currentPreset));
            setManualSpeed(shooterSpeeds.get(currentPreset));
        } else {
            setHoodPosition(0.0);
            setManualSpeed(0.0);
        }

        runServo();
        updateFlywheelSpeed();
    }

    public boolean isSpunUp(double tolerance) {
        return (MathUtil.isNear(automaticSpeed, flywheelMotor1.getVelocity().getValueAsDouble(), tolerance));
    }

    public boolean isSpunUp() {
        return isSpunUp(ShooterConstants.SPEED_TOLERANCE);
    }

}
