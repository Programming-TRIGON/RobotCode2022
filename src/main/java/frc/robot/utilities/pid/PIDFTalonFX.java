package frc.robot.utilities.pid;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.components.TrigonTalonFX;
import frc.robot.utilities.MotorConfig;

public class PIDFTalonFX extends TrigonTalonFX implements PIDFTalon {
    private PIDFCoefs pidfCoefs;
    private final ControlMode controlMode;
    private boolean isTuning;
    private double tuningSetpoint;

    /**
     * Constructs a new PIDF motor controller
     *
     * @param id          device ID of motor controller
     * @param motorConfig The configuration preset to use
     */
    public PIDFTalonFX(int id, MotorConfig motorConfig, ControlMode controlMode) {
        super(id, motorConfig);

        setCoefs(motorConfig.getCoefs());
        this.isTuning = false;
        this.tuningSetpoint = 0;
        this.controlMode = controlMode;
    }

    @Override
    public void setIsTuning(boolean isTuning) {
        this.isTuning = isTuning;
    }

    @Override
    public PIDFCoefs getCoefs() {
        return pidfCoefs;
    }

    @Override
    public void setSetpoint(double setpoint) {
        setWithF(controlMode, setpoint);
    }

    @Override
    public void setPIDFCoefs(PIDFCoefs coefs) {
        pidfCoefs = coefs;
    }

    @Override
    public double getTuningSetpoint() {
        return tuningSetpoint;
    }

    @Override
    public void setTuningSetpoint(double setpoint) {
        tuningSetpoint = setpoint;
        setWithF(controlMode, setpoint);
    }

    @Override
    public boolean isTuning() {
        return isTuning;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        PIDFTalon.super.initSendable(builder);
    }
}
