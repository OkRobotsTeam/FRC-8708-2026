// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util;

import edu.wpi.first.math.controller.PIDController;

public class LoggedTuneablePID extends PIDController {

    // Tunable numbers
    private LoggedTunableNumber p;
    private LoggedTunableNumber i;
    private LoggedTunableNumber d;

    public LoggedTuneablePID(String name, double kP, double kI,
                             double kD)
    {
        this(name, kP, kI, kD, .02);
    }

    public LoggedTuneablePID(String name, double p, double i,
                             double d, double period)
    {
        super(p, i, d, period);

        // Tunable numbers for PID and motion gain constants
        this.p = new LoggedTunableNumber(name + "/kP", p);
        this.i = new LoggedTunableNumber(name + "/kI", i);
        this.d = new LoggedTunableNumber(name + "/kD", d);
    }

    public void updatePID()
    {
        // If changed, update controller constants from Tuneable Numbers
        if (p.hasChanged(hashCode())
            || i.hasChanged(hashCode())
            || d.hasChanged(hashCode())) {
            this.setPID(p.get(), i.get(), d.get());
        }
    }

}
