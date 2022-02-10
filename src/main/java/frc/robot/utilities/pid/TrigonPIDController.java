package frc.robot.utilities.pid;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableBuilder;

public class TrigonPIDController extends PIDController implements PIDConfigurable {
    private final PIDCoefs pidCoefs;
    private boolean isTuning;
    private double tuningSetpoint;

    public TrigonPIDController(PIDCoefs pidCoefs) {
        super(pidCoefs.getKP(), pidCoefs.getKI(), pidCoefs.getKD());
        super.setTolerance(pidCoefs.getTolerance(), pidCoefs.getDeltaTolerance());
        this.pidCoefs = pidCoefs;
        isTuning = false;
    }

    @Override
    public double getTuningSetpoint() {
        return tuningSetpoint;
    }

    @Override
    public void setTuningSetpoint(double tuningSetpoint) {
        this.tuningSetpoint = tuningSetpoint;
    }

    @Override
    public PIDCoefs getCoefs() {
        return pidCoefs;
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

    public void initSendable(SendableBuilder builder) {
        PIDConfigurable.super.initSendable(builder);
    }
}
