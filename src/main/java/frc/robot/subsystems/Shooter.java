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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import org.littletonrobotics.junction.Logger;

/**
 * Subsystem to control two TalonFX motors independently (or optionally make the
 * top motor follow the bottom).
 */
public class Shooter extends SubsystemBase {
//    public final LoggedTuneablePID rotationPID = new LoggedTuneablePID("/Shooter/RotationPID", ShooterConstants.ANGLER_P, ShooterConstants.ANGLER_I, ShooterConstants.ANGLER_D);
//    public final LoggedTuneablePID shooterPID = new LoggedTuneablePID("/Shooter/ShooterPID", ShooterConstants.FLYWHEEL_P, ShooterConstants.FLYWHEEL_I, ShooterConstants.FLYWHEEL_D);
    Encoder encoder = new Encoder(ShooterConstants.ENCODER_CHANNEL_A, ShooterConstants.ENCODER_CHANNEL_B, ShooterConstants.ENCODER_REVERSED, ShooterConstants.ENCODER_ENCODING_TYPE);

    private final TalonFX shooterMotor1;
    private final TalonFX shooterMotor2;
    private final TalonFX angler;
    private final TalonFX injector;
    private final TalonFX transfer;

    private final VelocityDutyCycle velocityDutyCycle = new VelocityDutyCycle(0);
    private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0);
    boolean isRunning = false;
    double speed = 0;
    private enum Mode
    {
        STOPPED,
        IDLING,
        SHOOTING,
        MANUAL,
        TRANSFERRING
    }
    private Mode mode = Mode.STOPPED;

    private double automaticSpeed = 0.0;
    private double zoneSpeed = 0.0;
    private double automaticAngle = 0.0;

    private final RobotState robotState = RobotState.getInstance();

    private double anglerPosition = 0;

    private double encoderOffset = 0;


    public Shooter() {
        shooterMotor1 = new TalonFX(ShooterConstants.MOTOR_1_ID);
        shooterMotor2 = new TalonFX(ShooterConstants.MOTOR_2_ID);
        angler = new TalonFX(ShooterConstants.ANGLER_ID);
        injector = new TalonFX(ShooterConstants.INJECTOR_ID);
        transfer = new TalonFX(ShooterConstants.TRANSFER_ID);
        var configs = new MotorOutputConfigs();
        configs.Inverted = InvertedValue.Clockwise_Positive;
        configs.NeutralMode = NeutralModeValue.Brake;

        shooterMotor1.getConfigurator().apply(configs);
        angler.getConfigurator().apply(configs);
        injector.getConfigurator().apply(configs);
        transfer.getConfigurator().apply(configs);

        configs.Inverted = InvertedValue.CounterClockwise_Positive;
        shooterMotor2.getConfigurator().apply(configs);
        shooterMotor2.setControl(new Follower(shooterMotor1.getDeviceID(), MotorAlignmentValue.Opposed));

        shooterMotor1.setPosition(0.0);
        shooterMotor2.setPosition(0.0);
        angler.setPosition(ShooterConstants.ANGLER_STARTING_POSITION);

        encoderOffset = encoder.getDistance();

        TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration(); // Start with factory defaults
        Slot0Configs slot0Configs = new Slot0Configs();
        slot0Configs.kP = ShooterConstants.FLYWHEEL_P;
        slot0Configs.kV = ShooterConstants.FLYWHEEL_V;
        slot0Configs.kD = ShooterConstants.FLYWHEEL_D;

        shooterMotor1.getConfigurator().apply(slot0Configs);
        shooterMotor2.getConfigurator().apply(slot0Configs);

        slot0Configs.kP = ShooterConstants.ANGLER_P;
        slot0Configs.kV = ShooterConstants.ANGLER_V;
        slot0Configs.kD = ShooterConstants.ANGLER_D;

        angler.getConfigurator().apply(slot0Configs);

//        slot0Configs.kP = ShooterConstants.TRANSFER_P;
//        slot0Configs.kV = ShooterConstants.TRANSFER_V;
//        slot0Configs.kD = ShooterConstants.TRANSFER_D;
    }

    public void setModeStopped () {
        mode = Mode.STOPPED;
        System.out.println("setModeStopped");
    }

    public void setModeIdling () {
        mode = Mode.IDLING;
        System.out.println("setModeIdling");

    }

    public void setModeShooting () {
        mode = Mode.SHOOTING;
        System.out.println("setModeShooting");

    }

    public void setModeManual () {
        mode = Mode.MANUAL;
        System.out.println("setModeManual");

    }

    public void setModeTransferring () {
        mode = Mode.TRANSFERRING;
        System.out.println("setModeTransferring");

    }

    public void toggleIdling () {
        switch (mode) {
            case STOPPED -> setModeIdling();
            case IDLING, SHOOTING, MANUAL, TRANSFERRING -> setModeStopped();
        }
    }


    public void setAnglerMotor(double power) {
        angler.set(power);
    }

    public void setInjectorMotor(double speed) {
        injector.setControl(dutyCycleOut.withOutput(speed));
    }

    public void setTransferMotor(double speed) {
        transfer.setControl(dutyCycleOut.withOutput(speed));
    }

    /**
     * Set both motors to the same percent output.
     */
    public void setBothPercent(double percent) {
        //shooterMotor2.setControl(new VelocityVoltage(percent));
        Logger.recordOutput("Shooter/FlywheelSpeed", percent);
        shooterMotor1.setControl(new VelocityVoltage(percent));
//        System.out.println("setting both percent to " + percent);

    }

    /**
     * Stop both motors (zero output).
     */
    public void stop() {
        setBothPercent(0.0);
    }

    public void run() {
        setBothPercent(0.5);
    }

    public void setSpeedIfRunning(double speed) {
        if (isRunning) {
            setBothPercent(speed);
        } else {
            setBothPercent(0);
        }
    }


    public void toggleIsRunning() {
        isRunning = !isRunning;
        setSpeedIfRunning(speed);
    }

    public void changeSpeed(double input) {
        speed = speed + input;
        if (speed > 1) {
            speed = 1;
        } else if (speed < -1) {
            speed = -1;
        }
        setSpeedIfRunning(speed);
    }

    public void faster() {
        changeSpeed(0.1);
    }

    public void slower() {
        changeSpeed(-0.1);
    }

    public void setAngle(double position) {
//        rotationPID.setSetpoint(angle * 360.0);
        System.out.println("setAngle" +  position);
        anglerPosition = position * ShooterConstants.MAXIMUM_ANGULAR_ROTATIONS;
        Logger.recordOutput("Shooter/Angler", anglerPosition);

    }

    public void toggleAngles (double firstAngle, double secondAngle) {
        if (!MathUtil.isNear((secondAngle * ShooterConstants.MAXIMUM_ANGULAR_ROTATIONS), anglerPosition, 0.05)) {
            setAngle(secondAngle);
        } else {
            setAngle(firstAngle);
        }
    }

    public void calculateAutomaticAngle () {
        Pose2d currentPose = robotState.getEstimatedPose();
        Translation2d target;
        double distance;
        if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
            target = FieldConstants.BLUE_GOAL_POSITION;
        } else {
            target = FieldConstants.RED_GOAL_POSITION;
        }
        distance = target.getDistance(currentPose.getTranslation());

        automaticAngle = MathUtil.clamp(distance * 20, 0, 100);
    }

    public void calculateAutomaticSpeed() {
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

    public void shoot () {
        if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
            if (robotState.getEstimatedPose().getMeasureX().compareTo(FieldConstants.BLUE_GOAL_POSITION.getMeasureX()) < 0) {
                zoneSpeed = automaticSpeed;
                if (mode == Mode.SHOOTING) {
                    setBothPercent(automaticSpeed);
                }
            } else {
                zoneSpeed = 100;
                if (mode == Mode.SHOOTING) {
                    setBothPercent(100);
                }
            }
        } else {
            if (robotState.getEstimatedPose().getMeasureX().compareTo(FieldConstants.RED_GOAL_POSITION.getMeasureX()) > 0) {
                zoneSpeed = automaticSpeed;
                if (mode == Mode.SHOOTING) {
                    setBothPercent(automaticSpeed);
                }
            } else {
                zoneSpeed = 100;
                if (mode == Mode.SHOOTING) {
                    setBothPercent(100);
                }
            }
        }
        if (mode == Mode.IDLING) {
            setBothPercent(0.6 * zoneSpeed);
        }
    }

    /**
     * Example command factory method.
     *
     * @return The autonomous command
     */
    public Command exampleMethodCommand() {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return runOnce(
                () -> {
                    /* one-time action goes here */
                });
    }


    /**
     * An example method querying a boolean state of the subsystem (for example, a digital sensor).
     *
     * @return value of some boolean subsystem state, such as a digital sensor.
     */
    public boolean exampleCondition() {
        // Query some boolean state, such as a digital sensor.
        return false;
    }


    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        calculateAutomaticSpeed();

        switch(mode) {
            case STOPPED -> stop();
            case IDLING, SHOOTING -> shoot();
            case MANUAL ->   setBothPercent(speed * 100);
        }
        angler.setControl(new PositionDutyCycle(anglerPosition - encoderOffset));

        if (MathUtil.isNear(automaticSpeed, shooterMotor1.getVelocity().getValueAsDouble(), ShooterConstants.SPEED_TOLERANCE) && (mode == Mode.SHOOTING)) {
            setInjectorMotor(0.6);
        } else {
            setInjectorMotor(0);
        }

        if (mode == Mode.SHOOTING) {
            setTransferMotor(0.6);
        } else {
            setTransferMotor(0);
        }

//        System.out.println("Target anglerPosition: " + (anglerPosition - encoderOffset) + " current anglerPosition: " +
//                angler.getPosition() + " encoder offset: " + encoderOffset);
    }


    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
