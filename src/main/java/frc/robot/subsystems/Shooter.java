package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.LoggedTuneablePID;
import frc.robot.Constants.ShooterConstants;

/**
 * Subsystem to control two TalonFX motors independently (or optionally make the
 * top motor follow the bottom).
 */
public class Shooter extends SubsystemBase {
    public final LoggedTuneablePID rotationPID = new LoggedTuneablePID("/Shooter/RotationPID", ShooterConstants.KP, ShooterConstants.KI, ShooterConstants.KD);

    private final TalonFX motor2;
    private final TalonFX angler;
    private final TalonFX motor1;
    private final DutyCycleOut m_dutyCycle = new DutyCycleOut(0);
    boolean isRunning = false;
    double speed = 0;

    public Shooter() {
        motor2 = new TalonFX(ShooterConstants.BOTTOM_ID);
        angler = new TalonFX(ShooterConstants.ANGLER_ID);
        motor1 = new TalonFX(ShooterConstants.TOP_ID);
        var configs = new MotorOutputConfigs();
        configs.Inverted = InvertedValue.Clockwise_Positive;
        configs.NeutralMode = NeutralModeValue.Brake;

        motor1.getConfigurator().apply(configs);
        angler.getConfigurator().apply(configs);

        configs.Inverted = InvertedValue.CounterClockwise_Positive;
        motor2.getConfigurator().apply(configs);

        motor1.setPosition(0.0);
        motor2.setPosition(0.0);
        angler.setPosition(ShooterConstants.ANGLER_STARTING_POSITION);

    }


    public void setMotors(double power) {
        angler.set(power);
    }

    /**
     * Set bottom motor output as percent (-1.0 .. 1.0).
     */
    public void setMotor2Percent(double percent) {
        motor2.setControl(m_dutyCycle.withOutput(percent));
    }

    /**
     * Set top motor output as percent (-1.0 .. 1.0).
     * If top was configured to follow bottom, calling this will override that behavior.
     */
    public void setMotor1Percent(double percent) {
        motor1.setControl(m_dutyCycle.withOutput(percent));
    }

    /**
     * Set both motors to the same percent output.
     */
    public void setBothPercent(double percent) {
        setMotor2Percent(percent);
        setMotor1Percent(percent);
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

    public void setAngle(double angle) {
        rotationPID.setSetpoint(angle * 360.0);

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

        double pidOutput = rotationPID.calculate(angler.getPosition().getValueAsDouble());
        setMotors(pidOutput);
    }


    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
