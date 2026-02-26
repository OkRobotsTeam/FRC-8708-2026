package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Subsystem to control two TalonFX motors independently (or optionally make the
 * top motor follow the Motor).
 */
public class Intake extends SubsystemBase {
    private static final int MOTOR_ID = 5;
    private static final int TICKS_PER_REV = 2048; // TalonFX integrated sensor

    private final TalonFX motor;
    //private final TalonFX top;
    private final DutyCycleOut m_dutyCycle = new DutyCycleOut(0);
    double intakeSpeed = 0.5;

    public Intake() {
        motor = new TalonFX(MOTOR_ID);
        var configs = new MotorOutputConfigs();
        configs.Inverted = InvertedValue.CounterClockwise_Positive;
        configs.NeutralMode = NeutralModeValue.Brake;

        motor.getConfigurator().apply(configs);

        configs.Inverted = InvertedValue.CounterClockwise_Positive;

        motor.setPosition(0.0);
        
    }



    public void setMotorPercent(double percent) {
        motor.setControl(m_dutyCycle.withOutput(percent));
    }

    public void stop() {
        setMotorPercent(0.0);
    }

    public void run() {
        setMotorPercent(intakeSpeed);
        System.out.println("Running Intake");
    }

    public void runSpeed(Double speed) {
        setMotorPercent(speed);
        System.out.println("Running Intake");
    }

    public Command stopCommand() {
        return new InstantCommand(this::stop,this);
    }

    public Command runCommand() {
        return new InstantCommand(this::run,this);
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
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
