package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Meters;

/**
 * Subsystem to control two TalonFX motors independently (or optionally make the
 * top motor follow the bottom).
 */
public class Shooter extends SubsystemBase {
//    public final LoggedTuneablePID rotationPID = new LoggedTuneablePID("/Shooter/RotationPID", ShooterConstants.ANGLER_P, ShooterConstants.ANGLER_I, ShooterConstants.ANGLER_D);
//    public final LoggedTuneablePID shooterPID = new LoggedTuneablePID("/Shooter/ShooterPID", ShooterConstants.FLYWHEEL_P, ShooterConstants.FLYWHEEL_I, ShooterConstants.FLYWHEEL_D);
    Encoder encoder = new Encoder(ShooterConstants.ENCODER_CHANNEL_A, ShooterConstants.ENCODER_CHANNEL_B, ShooterConstants.ENCODER_REVERSED, ShooterConstants.ENCODER_ENCODING_TYPE);

    private final TalonFX flywheelMotor1;
    private final TalonFX flywheelMotor2;
    private final TalonFX hood;
    private final TalonFX injector;
    private final TalonFX transfer;

    private final VelocityDutyCycle velocityDutyCycle = new VelocityDutyCycle(0);
    private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0);
    boolean isRunning = false;



    boolean isShooting = false;
    public boolean idleWhenNotShooting;
    public boolean autoSpeedMode;
    public boolean autoHoodAngle;

    public double motorSpeed = 0;
    public double manualSpeed = 0;
    private double automaticSpeed = 0.0;

    private final RobotState robotState = RobotState.getInstance();

    public double automaticHoodPosition = 0.0;
    public double hoodMotorPosition = 0;
    public double hoodPosition = 0;
    public double manualHoodPosition = 0;

    private double encoderOffset = 0;


    public Shooter() {
        flywheelMotor1 = new TalonFX(ShooterConstants.MOTOR_1_ID);
        flywheelMotor2 = new TalonFX(ShooterConstants.MOTOR_2_ID);
        hood = new TalonFX(ShooterConstants.HOOD_ID);
        injector = new TalonFX(ShooterConstants.INJECTOR_ID);
        transfer = new TalonFX(ShooterConstants.TRANSFER_ID);
        var configs = new MotorOutputConfigs();
        configs.Inverted = InvertedValue.Clockwise_Positive;
        configs.NeutralMode = NeutralModeValue.Brake;

        flywheelMotor1.getConfigurator().apply(configs);
        hood.getConfigurator().apply(configs);
        injector.getConfigurator().apply(configs);
        transfer.getConfigurator().apply(configs);

        configs.Inverted = InvertedValue.CounterClockwise_Positive;
        flywheelMotor2.getConfigurator().apply(configs);
        flywheelMotor2.setControl(new Follower(flywheelMotor1.getDeviceID(), MotorAlignmentValue.Opposed));

        flywheelMotor1.setPosition(0.0);
        flywheelMotor2.setPosition(0.0);
        hood.setPosition(ShooterConstants.HOOD_STARTING_POSITION);

        encoderOffset = encoder.getDistance();

        TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration(); // Start with factory defaults
        Slot0Configs slot0Configs = new Slot0Configs();
        slot0Configs.kP = ShooterConstants.FLYWHEEL_P;
        slot0Configs.kV = ShooterConstants.FLYWHEEL_V;
        slot0Configs.kD = ShooterConstants.FLYWHEEL_D;

        flywheelMotor1.getConfigurator().apply(slot0Configs);
        flywheelMotor2.getConfigurator().apply(slot0Configs);

        slot0Configs.kP = ShooterConstants.HOOD_P;
        slot0Configs.kV = ShooterConstants.HOOD_V;
        slot0Configs.kD = ShooterConstants.HOOD_D;

        hood.getConfigurator().apply(slot0Configs);

//        slot0Configs.kP = ShooterConstants.TRANSFER_P;
//        slot0Configs.kV = ShooterConstants.TRANSFER_V;
//        slot0Configs.kD = ShooterConstants.TRANSFER_D;
    }

    public void setShooterModeStopped() {
        idleWhenNotShooting = false;
        System.out.println("setShooterModeStopped");
    }

    public void setShooterModeIdling() {
        idleWhenNotShooting = true;
        System.out.println("setShooterModeIdling");
        updateFlywheelSpeed();
    }

    public void toggleIdling () {
        System.out.println("toggleIdling");
        idleWhenNotShooting = !idleWhenNotShooting;
        updateFlywheelSpeed();
    }

    public void setShooterModeShooting() {
        isShooting = true;
        System.out.println("setShooterModeShooting");
    }

    public void setShooterModeManual() {
        autoSpeedMode = false;
        System.out.println("setShooterModeManual");

    }

    public void setHoodModeAutomatic() {
        autoSpeedMode = true;
        System.out.println("setHoodModeAutomatic");
    }

    public void setHoodModeManual() {
        autoHoodAngle = false;
        System.out.println("setHoodModeManual");
    }

    public void toggleAutoEverything () {
        if (autoSpeedMode || autoHoodAngle) {
            autoSpeedMode = false;
            autoHoodAngle = false;
        } else {
            autoSpeedMode = true;
            autoHoodAngle = true;
        }
        updateHoodAngle();
        updateFlywheelSpeed();
    }

    public void setInjectorMotor(double speed) {
        injector.setControl(dutyCycleOut.withOutput(speed));
    }

    public void setTransferMotor(double speed) {
        transfer.setControl(dutyCycleOut.withOutput(speed));
    }

    public void setManualSpeed(double input) {
        manualSpeed = MathUtil.clamp(input, 0, 1);
        updateFlywheelSpeed();
    }

    public void setManualAngle(double input) {
        manualHoodPosition = MathUtil.clamp(input, 0, 1);
        updateHoodAngle();
    }

    public void faster() {
        setManualSpeed(manualSpeed + 0.05);
    }

    public void slower() {
        setManualSpeed(manualSpeed - 0.05);
    }

    public void angleUp() {
        setManualAngle(manualHoodPosition + .1);
    }

    public void angleDown() {
        setManualAngle(manualHoodPosition - 0.1);
    }

    public void updateHoodAngle() {
        if (isShooting) {
            if (autoHoodAngle) {
                autoCalculateHoodAngle();
                hoodPosition = automaticHoodPosition;
            } else {
                hoodPosition = manualHoodPosition;
            }
        } else {
            hoodPosition = 0;
        }
        hoodPosition = MathUtil.clamp(hoodPosition, 0, 1);
        hoodMotorPosition = hoodPosition * ShooterConstants.MAXIMUM_ANGULAR_ROTATIONS;
        hood.setControl(new PositionDutyCycle(hoodMotorPosition - encoderOffset));
        Logger.recordOutput("Shooter/Angler", hoodMotorPosition);

    }

    public void autoCalculateHoodAngle() {
        Pose2d currentPose = robotState.getEstimatedPose();
        Translation2d target;
        double distance;
        if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
            target = FieldConstants.BLUE_GOAL_POSITION;
        } else {
            target = FieldConstants.RED_GOAL_POSITION;
        }
        distance = target.getDistance(currentPose.getTranslation());

        automaticHoodPosition = MathUtil.clamp(distance * 20, 0, 100);
    }

    public void updateFlywheelSpeed() {
        if (autoSpeedMode) {
            calculateAutomaticFlywheelSpeed();
            if (inOurZone(robotState.getEstimatedPose())) {
                if (isShooting) {
                    setFlywheelSpeedPercent(automaticSpeed);
                } else if (idleWhenNotShooting) {
                    setFlywheelSpeedPercent(automaticSpeed * 0.6);
                } else {
                    setFlywheelSpeedPercent(0);
                }
            } else {
                if (isShooting) {
                    setFlywheelSpeedPercent(ShooterConstants.SPEED_WHEN_OUTSIDE_ZONE);
                } else if (idleWhenNotShooting) {
                    setFlywheelSpeedPercent(ShooterConstants.SPEED_WHEN_OUTSIDE_ZONE * 0.6);
                } else {
                    setFlywheelSpeedPercent(0);
                }
            }
        } else {
            setFlywheelSpeedPercent(manualSpeed);
        }
    }

    /**
     * Set both motors to the same percent output.
     */
    public void setFlywheelSpeedPercent(double percent) {
        //shooterMotor2.setControl(new VelocityVoltage(percent));
        Logger.recordOutput("Shooter/FlywheelSpeed", percent);
        flywheelMotor1.setControl(new VelocityVoltage(percent));
        motorSpeed = percent;
//        System.out.println("setting both percent to " + percent);
    }

    public void calculateAutomaticFlywheelSpeed() {
        Pose2d currentPose = robotState.getEstimatedPose();
        Translation2d target;
        double distance;
        if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
            target = FieldConstants.BLUE_GOAL_POSITION;
        } else {
            target = FieldConstants.RED_GOAL_POSITION;
        }
        distance = target.getDistance(currentPose.getTranslation());

//        automaticSpeed = Math.atan2(target.getX(), target.getY()) - distance;
        automaticSpeed = MathUtil.clamp(distance * 20, 0, 100);

        Logger.recordOutput("Shooter/AutomaticSpeed", automaticSpeed);
    }

    @AutoLogOutput(key = "Shooter/CalculatedShootingPosition")
    public Translation2d calculateShootingPosition () {
        Pose2d currentPose = robotState.getEstimatedPose();
        if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
            if (! inBlueAllianceZone(currentPose)) {
                if (inBlueLeftHalf(currentPose)) {
                    return FieldConstants.BLUE_LEFT_AIM_POSITION;
                } else {
                    return FieldConstants.BLUE_RIGHT_AIM_POSITION;
                }
            } else {
                return  FieldConstants.BLUE_GOAL_POSITION;
            }
        } else if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
            if (! inRedAllianceZone(currentPose)) {
                if (inRedLeftHalf(currentPose)) {
                    return FieldConstants.RED_LEFT_AIM_POSITION;
                } else {
                    return FieldConstants.RED_RIGHT_AIM_POSITION;
                }
            } else {
                return FieldConstants.RED_GOAL_POSITION;
            }
        }
        return new Translation2d(Meters.of(100), Meters.of(100));
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

    public void manualShoot(double speed, double hoodAngle) {
        isShooting = true;
        autoSpeedMode = false;
        autoHoodAngle = false;
        updateHoodAngle();
        setManualSpeed(speed);
    }

    public void shoot () {
        isShooting = true;
        setTransferMotor(0.6);
        updateFlywheelSpeed();
        updateHoodAngle();
    }

    public void stopShooting() {
        isShooting = false;
        updateFlywheelSpeed();
        updateHoodAngle();
        setTransferMotor(0);
    }

    public boolean inOurZone(Pose2d currentPose) {
        if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue){
            return inBlueAllianceZone(currentPose);
        } else {
            return inRedAllianceZone(currentPose);
        }
    }


    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        Translation2d shootingPosition = calculateShootingPosition();
        Logger.recordOutput("Shooter/shootingPosition", shootingPosition);

        if (isShooting && isSpunUp()) {
            setInjectorMotor(0.6);
        } else {
            setInjectorMotor(0);
        }

        if (autoSpeedMode) {
            updateFlywheelSpeed();
        }
        if (autoHoodAngle && isShooting) {
            autoCalculateHoodAngle();
            updateHoodAngle();
        }

    }

    public boolean isSpunUp() {
        return (MathUtil.isNear(automaticSpeed, flywheelMotor1.getVelocity().getValueAsDouble(), ShooterConstants.SPEED_TOLERANCE));
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
