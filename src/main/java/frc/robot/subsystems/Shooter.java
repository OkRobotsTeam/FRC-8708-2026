package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Subsystem to control two TalonFX motors independently (or optionally make the
 * top motor follow the bottom).
 */
public class Shooter extends SubsystemBase {
    private static final int BOTTOM_ID = 1;
    private static final int TOP_ID = 2;
    private static final int TICKS_PER_REV = 2048; // TalonFX integrated sensor

    private final TalonFX bottom;
    private final TalonFX top;
    private final DutyCycleOut m_dutyCycle = new DutyCycleOut(0);
    boolean isRunning = false;
    double topSpeed = 0;
    double bottomSpeed = 0;

    public Shooter() {
        bottom = new TalonFX(BOTTOM_ID);
        top = new TalonFX(TOP_ID);
        var configs = new MotorOutputConfigs();
        configs.Inverted = InvertedValue.Clockwise_Positive;
        configs.NeutralMode = NeutralModeValue.Brake;

        top.getConfigurator().apply(configs);

        configs.Inverted = InvertedValue.CounterClockwise_Positive;
        bottom.getConfigurator().apply(configs);

        top.setPosition(0.0);
        bottom.setPosition(0.0);
    }

    /**
     * Set bottom motor output as percent (-1.0 .. 1.0).
     */
    public void setBottomPercent(double percent) {
        bottom.setControl(m_dutyCycle.withOutput(percent));
    }

    /**
     * Set top motor output as percent (-1.0 .. 1.0).
     * If top was configured to follow bottom, calling this will override that behavior.
     */
    public void setTopPercent(double percent) {
        top.setControl(m_dutyCycle.withOutput(percent));
    }

    /**
     * Set both motors to the same percent output.
     */
    public void setBothPercent(double percent) {
        setBottomPercent(percent);
        setTopPercent(percent);
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

    public void setTopPercentIfRunning(double speed) {
        if (isRunning) {
            setTopPercent(speed);
        } else {
            setTopPercent(0);
        }
    }

    public void setBottomPercentIfRunning(double speed) {
        if (isRunning) {
            setBottomPercent(speed);
        } else {
            setBottomPercent(0);
        }
    }

    public void toggleIsRunning() {
        isRunning = !isRunning;
        setTopPercentIfRunning(topSpeed);
        setBottomPercentIfRunning(bottomSpeed);
    }

    public void changeTopSpeed(double input) {
        topSpeed = topSpeed + input;
        if (topSpeed > 1) {
            topSpeed = 1;
        } else if (topSpeed < -1) {
            topSpeed = -1;
        }
        setTopPercentIfRunning(topSpeed);
    }

    public void changeBottomSpeed(double input) {
        bottomSpeed = bottomSpeed + input;
        if (bottomSpeed > 1) {
            bottomSpeed = 1;
        } else if (bottomSpeed < -1) {
            bottomSpeed = -1;
        }
        setBottomPercentIfRunning(bottomSpeed);
    }

    public void topFaster() {
        changeTopSpeed(0.1);
    }

    public void topSlower() {
        changeTopSpeed(-0.1);
    }

    public void bottomFaster() {
        changeBottomSpeed(0.1);
    }

    public void bottomSlower() {
        changeBottomSpeed(-0.1);
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
    }


    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
