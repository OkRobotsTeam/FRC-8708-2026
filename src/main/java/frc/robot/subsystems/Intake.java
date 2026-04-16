package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.LoggedTuneablePID;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import org.littletonrobotics.junction.Logger;

/**
 * Subsystem to control two TalonFX motors independently (or optionally make the
 * top motor follow the Motor).
 */
public class Intake extends SubsystemBase {


    public final LoggedTuneablePID positionPID = new LoggedTuneablePID("/Intake/PositionPID", IntakeConstants.KP, IntakeConstants.KI, IntakeConstants.KD);

    private final TalonFX intakeMotor;
    private final TalonFX intakeExtender;
    private final DutyCycleOut m_dutyCycle = new DutyCycleOut(0);
    public double intakeSpeed = 0.7;
    public int currentState = 0;
    private double targetPosition = 0;
    private boolean shooting = false;
    private double encoderOffset = 0;
    private enum IntakeState {FORWARDS, STOPPED, BACKWARDS};
    IntakeState intakeState = IntakeState.STOPPED;
    Encoder encoder = new Encoder(IntakeConstants.ENCODER_CHANNEL_A, IntakeConstants.ENCODER_CHANNEL_B, IntakeConstants.ENCODER_REVERSED, IntakeConstants.ENCODER_ENCODING_TYPE);

    public Intake() {
        intakeMotor = new TalonFX(IntakeConstants.INTAKE_MOTOR_ID);
        intakeExtender = new TalonFX(IntakeConstants.INTAKE_EXTENDER_ID);
        var configs = new MotorOutputConfigs();
        configs.NeutralMode = NeutralModeValue.Brake;

        // APPLY CURRENT LIMITS
        intakeMotor.getConfigurator().apply(Constants.INTAKE_WHEEL_CURRENT_LIMITS);
        intakeExtender.getConfigurator().apply(Constants.INTAKE_EXTENDER_CURRENT_LIMITS);


        configs.Inverted = InvertedValue.Clockwise_Positive;
        intakeMotor.getConfigurator().apply(configs);

        configs.PeakForwardDutyCycle = 0.15;
        configs.PeakReverseDutyCycle = -0.1;
        configs.Inverted = InvertedValue.CounterClockwise_Positive;


//        var rampConfig = new TalonFXConfiguration();
//        rampConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.2;
//        rampConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.2;
//        intakeExtender.getConfigurator().apply(rampConfig);

        configs.NeutralMode = NeutralModeValue.Brake;
        intakeExtender.getConfigurator().apply(configs);

        intakeMotor.setPosition(0.0);
        intakeExtender.setPosition(0.0);


        TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration(); // Start with factory defaults
        Slot0Configs slot0Configs = new Slot0Configs();
        slot0Configs.kP = IntakeConstants.EXTENDER_KP;
        slot0Configs.kV = IntakeConstants.EXTENDER_KV;
        slot0Configs.kD = IntakeConstants.EXTENDER_KD;

        intakeExtender.getConfigurator().apply(slot0Configs);


        slot0Configs.kP = Constants.IntakeConstants.KP;
        slot0Configs.kV = Constants.IntakeConstants.KV;
        slot0Configs.kD = Constants.IntakeConstants.KD;
        intakeMotor.getConfigurator().apply(slot0Configs);


        encoder.reset();

        encoderOffset = encoder.getDistance();
        
    }


    public void extendIntake() {
//        positionPID.setSetpoint(IntakeConstants.EXTENDED_POSITION);
        targetPosition = IntakeConstants.EXTENDED_POSITION;
        currentState = 1;
        positionPID.setSetpoint(IntakeConstants.EXTENDED_POSITION);
    }

    public void retractIntake() {
//        positionPID.setSetpoint(IntakeConstants.RETRACTED_POSITION);
        targetPosition = IntakeConstants.RETRACTED_POSITION;
        currentState = 0;
        positionPID.setSetpoint(IntakeConstants.RETRACTED_POSITION);
    }

    public void toggleIntake() {
        if (currentState == 0) {
            extendIntake();
        } else if (currentState == 1) {
            retractIntake();
        }
    }

    public void setShooting() {
        shooting = true;
    }

    public void stopShooting() {
        shooting = false;
    }

    public void setIntakeSpeed(double percent) {
        double speed = percent * 100.0;
//        System.out.println("Setting speed to" + speed);
        intakeMotor.setControl(new VelocityVoltage(speed));
    }


    public void stop() {
        intakeState = IntakeState.STOPPED;
        System.out.println("Stopping Intake");
    }

    public void run() {
        extendIntake();
        intakeState = IntakeState.FORWARDS;
        System.out.println("Running Intake");
    }

    public void runBackwards() {
        extendIntake();
        intakeState = IntakeState.BACKWARDS;
        System.out.println("Running Intake Backwards");
    }

    public void runSpeed(Double speed) {
        setIntakeSpeed(speed);
//        System.out.println("Running Intake" + speed);
    }

    public void changeMotorSpeed(double input) {
        intakeSpeed = intakeSpeed + input;
        if (intakeSpeed > 1) {
            intakeSpeed = 1;
        } else if (intakeSpeed < -1) {
            intakeSpeed = -1;
        }
//        System.out.println("Setting intake speed to :" + intakeSpeed);
    }

    public void setExtenderTarget(double position) {
        double currentEncoderPosition = encoder.getDistance();
        double currentMotorPosition = intakeExtender.getPosition().getValueAsDouble();
        targetPosition = MathUtil.clamp(targetPosition, 0, IntakeConstants.EXTENDED_POSITION);
        double encoderError = targetPosition - currentEncoderPosition;
        double motorError = encoderError * 0.008;

        if (Math.abs(encoderError) > 100) {
            intakeExtender.setControl(new PositionDutyCycle(motorError + currentMotorPosition));
//            System.out.println("Target Position: " +  targetPosition + " current encoder position: "
//                    + currentEncoderPosition + " current motor position: " + currentMotorPosition + " target motor position: " + (motorError + currentMotorPosition));
        }

    }

    public void faster() {
        changeMotorSpeed(0.1);
    }

    public void slower() {
        changeMotorSpeed(-0.1);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

//        double pidOutput = positionPID.calculate(intakeActuator.getPosition().getValueAsDouble());
//        setMotors(pidOutput);

        //System.out.println("Target position: " + targetPosition + " Current position: " + intakeActuator.getPosition());

        if (shooting & (encoder.get() > 100 )) {

            targetPosition = targetPosition - 10;
        }

        setExtenderTarget(targetPosition);

        if (encoder.getDistance() < 600) {
            runSpeed(0.0);
        } else
//            System.out.println("IS: " + intakeState.name());
            switch (intakeState) {
                case STOPPED -> runSpeed(0.0);
                case FORWARDS -> runSpeed(intakeSpeed);
                case BACKWARDS -> runSpeed(-intakeSpeed);
        }

        Logger.recordOutput("Intake/Extender Speed", intakeExtender.getVelocity().getValueAsDouble());
        Logger.recordOutput("Intake/Motor Speed", intakeMotor.getVelocity().getValueAsDouble());



//        System.out.println("Position: " +  encoder.getDistance() / 2048 + " PID Target: " + positionPID.getSetpoint() + " PID Output: " + pidOutput);

    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
