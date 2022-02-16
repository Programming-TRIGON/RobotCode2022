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

    default void setWithF(ControlMode controlMode, double setpoint, boolean isTuning) {
        // If we're tuning and the setpoint is not the tuning setpoint, then we ignore and don't change the setpoint
        if(isTuning() && !isTuning)
            return;
        set(controlMode, setpoint, DemandType.ArbitraryFeedForward, getCoefs().getKS());
    }

    default void setSetpoint(double setpoint) {
        setWithF(getControlMode(), setpoint, false);
    }

    default void setTuningSetpoint(double setpoint) {
        setWithF(getControlMode(), setpoint, true);
    }

    double getClosedLoopTarget();

    default double getSetpoint() {
        boolean closedLoop = getControlMode() == ControlMode.Position || getControlMode() == ControlMode.Velocity;

        return closedLoop ? getClosedLoopTarget() : 0;
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
