package frc.robot.utilities.pid;

import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.utilities.Saveable;

public interface PIDConfigurable extends Saveable {

    PIDCoefs getCoefs();

    default void setCoefs(PIDCoefs coefs) {
        getCoefs().set(coefs);
    }

    PIDCoefs getRemoteCoefs();

    default void setRemoteCoefs(PIDCoefs coefs) {
        getRemoteCoefs().set(coefs);
    }

    default PIDCoefs getRemoteCoefsCopy() {
        return new PIDCoefs(getRemoteCoefs());
    }

    double getSetpoint();

    void setSetpoint(double setpoint, boolean isTuning);

    default void setSetpoint(double setpoint) {
        setSetpoint(setpoint, false);
    }

    boolean isTuning();

    void setIsTuning(boolean isTuning);

    default double getKP() {
        return getCoefs().getKP();
    }

    void setKP(double KP);

    default double getKI() {
        return getCoefs().getKI();
    }

    void setKI(double KI);

    default double getKD() {
        return getCoefs().getKD();
    }

    void setKD(double KD);

    default double getTolerance() {
        return getCoefs().getTolerance();
    }

    void setTolerance(double tolerance);

    default double getDeltaTolerance() {
        return getCoefs().getDeltaTolerance();
    }

    void setDeltaTolerance(double deltaTolerance);

    @Override
    default void save() {
        setRemoteCoefs(getCoefs());
    }

    @Override
    default void load() {
        setCoefs(getRemoteCoefsCopy());
    }

    @Override
    default void initSendable(SendableBuilder builder) {
        Saveable.super.initSendable(builder);
        // sends the pid values to the dashboard but only allows them to be changed if
        // isTuning is true
        builder.setSmartDashboardType("PIDConfigurable");
        builder.addDoubleProperty("p", this::getKP, kP -> setKP(isTuning() ? kP : getKP()));
        builder.addDoubleProperty("i", this::getKI, kI -> setKI(isTuning() ? kI : getKI()));
        builder.addDoubleProperty("d", this::getKD, kD -> setKD(isTuning() ? kD : getKD()));
        builder.addDoubleProperty(
                "tolerance", getCoefs()::getTolerance,
                tolerance -> setTolerance(isTuning() ? tolerance : getTolerance()));
        builder.addDoubleProperty(
                "deltaTolerance", getCoefs()::getDeltaTolerance,
                deltaTolerance -> setDeltaTolerance(isTuning() ? deltaTolerance : getDeltaTolerance()));
        builder.addDoubleProperty("setpoint", this::getSetpoint,
                (setpoint) -> {
                    if(isTuning())
                        setSetpoint(setpoint, true);
                });
        builder.addBooleanProperty("isTuning", this::isTuning, this::setIsTuning);
    }
}
