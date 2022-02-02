package frc.robot.utilities.pid;

import edu.wpi.first.util.sendable.SendableBuilder;

public interface PIDFConfigurable extends PIDConfigurable {

    @Override
    PIDFCoefs getCoefs();

    default double getKV() {
        return getCoefs().getKV();
    }

    void setKV(double v);

    default double getKS() {
        return getCoefs().getKS();
    }

    void setKS(double s);

    @Override
    default void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("s", this::getKS, kS -> setKS(isTuning() ? kS : getKS()));
        builder.addDoubleProperty("v", this::getKV, kV -> setKV(isTuning() ? kV : getKV()));
        PIDConfigurable.super.initSendable(builder);
    }
}
