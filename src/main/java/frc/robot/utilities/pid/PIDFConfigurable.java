package frc.robot.utilities.pid;

import edu.wpi.first.util.sendable.SendableBuilder;

public interface PIDFConfigurable extends PIDConfigurable {

    @Override
    PIDFCoefs getCoefs();

    default void setCoefs(PIDFCoefs coefs) {
        getCoefs().set(coefs);
    }

    PIDFCoefs getRemoteCoefs();

    default void setRemoteCoefs(PIDFCoefs coefs) {
        getRemoteCoefs().set(coefs);
    }

    @Override
    default PIDFCoefs getRemoteCoefsCopy() {
        return new PIDFCoefs(getRemoteCoefs());
    }

    default double getKV() {
        return getCoefs().getKV();
    }

    void setKV(double v);

    default double getKS() {
        return getCoefs().getKS();
    }

    void setKS(double s);

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
        PIDConfigurable.super.initSendable(builder);
        builder.setSmartDashboardType("PIDFConfigurable");
        builder.addDoubleProperty("s", this::getKS, kS -> setKS(isTuning() ? kS : getKS()));
        builder.addDoubleProperty("v", this::getKV, kV -> setKV(isTuning() ? kV : getKV()));
    }
}
