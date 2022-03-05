package frc.robot.utilities.pid;

import edu.wpi.first.util.sendable.SendableBuilder;

/**
 * A class that can be configured with a PIDFConfig.
 */
public interface PIDFConfigurable extends PIDConfigurable {

    @Override
    PIDFCoefs getCoefs();

    /**
     * Sets the pidfCoefs values
     */
    default void setCoefs(PIDFCoefs coefs) {
        getCoefs().set(coefs);
    }

    /**
     * @return the PIDFCoefs that is inside LOCAL_CONSTANTS
     */

    PIDFCoefs getRemoteCoefs();

    /**
     * Sets the PIDF values inside LOCAL_CONSTANTS
     *
     * @param coefs the PIDFCoefs to set to
     */
    default void setRemoteCoefs(PIDFCoefs coefs) {
        getRemoteCoefs().set(coefs);
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
        setCoefs(getRemoteCoefs());
    }

    @Override
    default void initSendable(SendableBuilder builder) {
        PIDConfigurable.super.initSendable(builder);
        builder.setSmartDashboardType("PIDFConfigurable");
        builder.addDoubleProperty("s", this::getKS, kS -> setKS(isTuning() ? kS : getKS()));
        builder.addDoubleProperty("v", this::getKV, kV -> setKV(isTuning() ? kV : getKV()));
    }
}
