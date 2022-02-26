package frc.robot.utilities.pid;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableBuilder;

public class TrigonPIDController extends PIDController implements PIDConfigurable {
    private final PIDCoefs pidCoefs;
    private final PIDCoefs remotePidCoefs;
    private boolean isTuning;

    public TrigonPIDController(PIDCoefs coefs) {
        super(coefs.getKP(), coefs.getKI(), coefs.getKD());

        remotePidCoefs = coefs;
        pidCoefs = new PIDCoefs(getRemoteCoefs());
        isTuning = false;
    }

    @Override
    public void setSetpoint(double setpoint, boolean isTuning) {
        // If the controller is tuning, and whatever called the function
        // has nothing to do with tuning, we ignore the calling.
        if(isTuning() && !isTuning)
            return;
        super.setSetpoint(setpoint);
    }

    public void setSetpoint(double setpoint) {
        setSetpoint(setpoint, false);
    }

    @Override
    public PIDCoefs getCoefs() {
        return pidCoefs;
    }

    @Override
    public PIDCoefs getRemoteCoefs() {
        return remotePidCoefs;
    }

    @Override
    public boolean isTuning() {
        return isTuning;
    }

    @Override
    public void setIsTuning(boolean isTuning) {
        this.isTuning = isTuning;
    }

    @Override
    public void setKP(double p) {
        pidCoefs.setKP(p);
        super.setP(p);
    }

    @Override
    public void setKI(double i) {
        pidCoefs.setKI(i);
        super.setI(i);
    }

    @Override
    public void setKD(double d) {
        pidCoefs.setKD(d);
        super.setD(d);
    }

    @Override
    public void setTolerance(double tolerance) {
        pidCoefs.setTolerance(tolerance);
        super.setTolerance(tolerance);
    }

    @Override
    public void setDeltaTolerance(double deltaTolerance) {
        pidCoefs.setDeltaTolerance(deltaTolerance);
        super.setTolerance(pidCoefs.getTolerance(), deltaTolerance);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        PIDConfigurable.super.initSendable(builder);
    }
}
