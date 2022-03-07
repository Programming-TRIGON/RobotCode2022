package frc.robot.utilities.pid;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

// This does not implement pidConfigurable because this classes setpoint clashes with the interfaces
public class TrigonProfiledPIDController extends ProfiledPIDController implements Sendable {
    private PIDCoefs pidCoefs;
    private boolean isTuning;

    public TrigonProfiledPIDController(PIDCoefs pidCoefs, TrapezoidProfile.Constraints constraints) {
        super(pidCoefs.getKP(), pidCoefs.getKI(), pidCoefs.getKD(), constraints);
        this.pidCoefs = pidCoefs;
        super.setTolerance(pidCoefs.getTolerance(), pidCoefs.getDeltaTolerance());
    }

    public void setSetpoint(double goal, boolean isTuning) {
        // If the controller is tuning, and whatever called the function
        // has nothing to do with tuning, we ignore the calling.
        if(isTuning() && !isTuning)
            return;
        super.setGoal(goal);
    }

    public boolean isTuning() {
        return isTuning;
    }

    public void setIsTuning(boolean isTuning) {
        this.isTuning = isTuning;
    }

    public void setKP(double p) {
        pidCoefs.setKP(p);
        super.setP(p);
    }

    public void setKI(double i) {
        pidCoefs.setKI(i);
        super.setI(i);
    }

    public void setKD(double d) {
        pidCoefs.setKD(d);
        super.setD(d);
    }

    public double getTolerance() {
        return getCoefs().getTolerance();
    }

    @Override
    public void setTolerance(double tolerance) {
        pidCoefs.setTolerance(tolerance);
        super.setTolerance(tolerance);
    }

    public double getDeltaTolerance() {
        return getCoefs().getDeltaTolerance();
    }

    public void setDeltaTolerance(double deltaTolerance) {
        pidCoefs.setDeltaTolerance(deltaTolerance);
        super.setTolerance(pidCoefs.getTolerance(), deltaTolerance);
    }

    public PIDCoefs getCoefs() {
        return pidCoefs;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        // sends the pid values to the dashboard but only allows them to be changed if
        // isTuning is true
        builder.setSmartDashboardType("PIDConfigurable");
        builder.addDoubleProperty("p", this::getP, kP -> setKP(isTuning() ? kP : getP()));
        builder.addDoubleProperty("i", this::getI, kI -> setKI(isTuning() ? kI : getI()));
        builder.addDoubleProperty("d", this::getD, kD -> setKD(isTuning() ? kD : getD()));
        builder.addDoubleProperty(
                "tolerance", getCoefs()::getTolerance,
                tolerance -> setTolerance(isTuning() ? tolerance : getTolerance()));
        builder.addDoubleProperty(
                "deltaTolerance", getCoefs()::getDeltaTolerance,
                deltaTolerance -> setDeltaTolerance(isTuning() ? deltaTolerance : getDeltaTolerance()));
        builder.addDoubleProperty("setpoint", () -> getGoal().position,
                (setpoint) -> {
                    if(isTuning())
                        setSetpoint(setpoint, true);
                });
        builder.addBooleanProperty("isTuning", this::isTuning, this::setIsTuning);
    }
}
