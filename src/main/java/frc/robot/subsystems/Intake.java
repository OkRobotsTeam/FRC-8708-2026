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
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.LoggedTuneablePID;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

/**
 * Subsystem to control two TalonFX motors independently (or optionally make the
 * top motor follow the Motor).
 */
public class Intake extends SubsystemBase {


    public final LoggedTuneablePID positionPID = new LoggedTuneablePID("/Intake/PositionPID", IntakeConstants.KP, IntakeConstants.KI, IntakeConstants.KD);

    private final TalonFX intakeMotor;
    private final TalonFX intakeExtender;
    private final DutyCycleOut m_dutyCycle = new DutyCycleOut(0);
    double intakeSpeed = 1;
    public int currentState = 0;
    private double targetPosition = 0;
    private boolean wiggling = false;
    private double encoderOffset = 0;
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
        slot0Configs.kP = IntakeConstants.KP;
        slot0Configs.kV = IntakeConstants.KV;
        slot0Configs.kD = IntakeConstants.KD;

        intakeExtender.getConfigurator().apply(slot0Configs);

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
            runSpeed(intakeSpeed);
        } else if (currentState == 1) {
            retractIntake();
            runSpeed(0.0);
        }
    }

    public void wiggle() {
        wiggling = true;
    }

    public void stopWiggle() {
        wiggling = false;
    }

    public void setIntakeSpeed(double percent) {
        intakeMotor.setControl(m_dutyCycle.withOutput(percent));
    }

    public void stop() {
        setIntakeSpeed(0.0);
        System.out.println("Stopping Intake");
    }

    public void run() {
        setIntakeSpeed(intakeSpeed);
        System.out.println("Running Intake");
    }

    public void runSpeed(Double speed) {
        setIntakeSpeed(speed);
        System.out.println("Running Intake" + speed);
    }

    public void changeMotorSpeed(double input) {
        intakeSpeed = intakeSpeed + input;
        if (intakeSpeed > 1) {
            intakeSpeed = 1;
        } else if (intakeSpeed < -1) {
            intakeSpeed = -1;
        }
        System.out.println("Setting intake speed to :" + intakeSpeed);
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

        if (wiggling) {
            if (System.currentTimeMillis() % 500 > 250) {
                //intakeExtender.setControl(new VelocityDutyCycle(0));
            } else {
                //intakeExtender.setControl(new VelocityDutyCycle(IntakeConstants.WIGGLE_ROTATIONS));
            }
        }
        double pidOutput = MathUtil.clamp(positionPID.calculate(encoder.getDistance() / 2048), -0.15, 0.15);

        //intakeExtender.setControl(new VelocityVoltage(pidOutput * 100));
        intakeExtender.setControl(new VoltageOut(pidOutput));

        System.out.println("Position: " +  encoder.getDistance() / 2048 + " PID Target: " + positionPID.getSetpoint() + " PID Output: " + pidOutput);

    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
