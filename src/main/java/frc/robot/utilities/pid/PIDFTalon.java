package frc.robot.utilities.pid;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import frc.robot.utilities.ConfigurableTalon;

public interface PIDFTalon extends ConfigurableTalon, PIDFMotor {

    PIDFCoefs getCoefs();

    default void setCoefs(PIDFCoefs pidfCoefs) {
        setPIDFCoefs(pidfCoefs);
        setKP(pidfCoefs.getKP());
        setKI(pidfCoefs.getKI());
        setKD(pidfCoefs.getKD());
        setKV(pidfCoefs.getKV());
        setKS(pidfCoefs.getKS());
        setTolerance(pidfCoefs.getTolerance());
    }

    void setPIDFCoefs(PIDFCoefs coefs);

    double getTuningSetpoint();

    default void setWithF(ControlMode controlMode, double setpoint) {
        set(controlMode, setpoint, DemandType.ArbitraryFeedForward, getCoefs().getKS());
    }

    double getClosedLoopTarget();

    default double getSetpoint() {
        return isTuning() ? getTuningSetpoint() : getClosedLoopTarget();
    }

    boolean isTuning();

    default void setKP(double p) {
        getCoefs().setKP(p);
        ce_config_kP(0, p);
    }

    default void setKI(double i) {
        getCoefs().setKI(i);
        ce_config_kI(0, i);
    }

    default void setKD(double d) {
        getCoefs().setKD(d);
        ce_config_kD(0, d);
    }

    default void setKV(double v) {
        getCoefs().setKV(v);
        ce_config_kF(0, v);
    }

    default void setKS(double s) {
        getCoefs().setKS(s);
    }

    default void setTolerance(double tolerance) {
        getCoefs().setTolerance(tolerance);
        ce_configAllowableClosedloopError(0, (int) tolerance);
    }

    default void setDeltaTolerance(double deltaTolerance) {
        //  This does nothing
    }

    double get();

    void set(double value);
}
