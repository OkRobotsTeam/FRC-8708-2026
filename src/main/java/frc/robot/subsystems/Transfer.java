package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Subsystem to control two TalonFX motors independently (or optionally make the
 * top motor follow the bottom).
 */
public class Transfer extends SubsystemBase {
    private static final int BOTTOM_ID = 3;
    private static final int TOP_ID = 4;
    private static final int TICKS_PER_REV = 2048; // TalonFX integrated sensor

    private final TalonFX bottom;
    private final TalonFX top;
    private final DutyCycleOut m_dutyCycle = new DutyCycleOut(0);
    boolean isRunning = false;
    double speed = 0;


    public Transfer() {
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

    public void setPercentIfRunning (double speed) {
        if (isRunning) {
            setTopPercent(speed);
            setBottomPercent(speed);
        } else {
            setTopPercent(0);
            setBottomPercent(0);
        }
    }
    public void toggleIsRunning() {
        isRunning = !isRunning;

    }
    public void changeSpeed(double input) {
        speed = speed + input;
        if (speed > 1) {
            speed = 1;
        } else if (speed < -1) {
            speed = -1;
        }
        setPercentIfRunning(speed);
    }
    public void faster() {
        changeSpeed(0.1);
    }

    public void slower() {
        changeSpeed(-0.1);
    }
}