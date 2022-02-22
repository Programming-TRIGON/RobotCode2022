package frc.robot.utilities.pid;

import edu.wpi.first.util.sendable.SendableBuilder;

public class TrigonPIDFController extends TrigonPIDController implements PIDFConfigurable {
    private final PIDFCoefs pidfCoefs;
    private final PIDFCoefs remotePidfCoefs;

    public TrigonPIDFController(PIDFCoefs coefs) {
        super(coefs);

        remotePidfCoefs = coefs;
        pidfCoefs = getRemoteCoefsCopy();
    }

    /**
     * Returns the next output of the PID controller using Feed-Forward.
     *
     * @param measurement The current measurement of the process variable.
     */
    @Override
    public double calculate(double measurement) {
        return super.calculate(measurement) + getKV() * getSetpoint() + getKS() * Math.signum(getSetpoint());
    }

    /**
     * Sets the Velocity gain coefficient of the Feed-Forward controller gain.
     *
     * @param v Velocity gain Feed-Forward coefficient
     */
    @Override
    public void setKV(double v) {
        pidfCoefs.setKV(v);
    }

    /**
     * Sets the Static gain coefficient of the Feed-Forward controller gain.
     *
     * @param s Static gain Feed-Forward coefficient
     */
    @Override
    public void setKS(double s) {
        pidfCoefs.setKS(s);
    }

    @Override
    public PIDFCoefs getCoefs() {
        return pidfCoefs;
    }

    @Override
    public PIDFCoefs getRemoteCoefs() {
        return remotePidfCoefs;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        PIDFConfigurable.super.initSendable(builder);
    }
}
