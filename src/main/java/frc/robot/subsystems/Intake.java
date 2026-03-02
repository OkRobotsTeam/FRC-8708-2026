package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.LoggedTuneablePID;
import frc.robot.Constants.IntakeConstants;

/**
 * Subsystem to control two TalonFX motors independently (or optionally make the
 * top motor follow the Motor).
 */
public class Intake extends SubsystemBase {


    public final LoggedTuneablePID positionPID = new LoggedTuneablePID("/Intake/PositionPID", IntakeConstants.KP, IntakeConstants.KI, IntakeConstants.KD);

    private final TalonFX intakeMotor;
    private final TalonFX intakeActuator;
    private final DutyCycleOut m_dutyCycle = new DutyCycleOut(0);
    double intakeSpeed = 0.5;
    public int currentState = 0;

    public Intake() {
        intakeMotor = new TalonFX(IntakeConstants.PICKUP_MOTOR_ID);
        intakeActuator = new TalonFX(IntakeConstants.PICKUP_ACTUATOR_ID);
        var configs = new MotorOutputConfigs();
        configs.Inverted = InvertedValue.CounterClockwise_Positive;
        configs.NeutralMode = NeutralModeValue.Brake;

        intakeMotor.getConfigurator().apply(configs);
        intakeActuator.getConfigurator().apply(configs);

        configs.Inverted = InvertedValue.CounterClockwise_Positive;

        intakeMotor.setPosition(0.0);
        
    }



    public void extendIntake() {
        positionPID.setSetpoint(IntakeConstants.EXTENDED_POSITION);
        currentState = 1;
    }

    public void retractIntake() {
        positionPID.setSetpoint(IntakeConstants.RETRACTED_POSITION);
        currentState = 0;
    }


    public void setMotors(double power) {
        intakeActuator.set(power);
    }


    public void setMotorPercent(double percent) {
        intakeMotor.setControl(m_dutyCycle.withOutput(percent));
    }

    public void stop() {
        setMotorPercent(0.0);
        System.out.println("Stopping Intake");
    }

    public void run() {
        setMotorPercent(intakeSpeed);
        System.out.println("Running Intake");
    }

    public void runSpeed(Double speed) {
        setMotorPercent(speed);
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

        double pidOutput = positionPID.calculate(intakeActuator.getPosition().getValueAsDouble());
        setMotors(pidOutput);
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
