package frc.robot.utilities.pid;

import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.components.TrigonTalonSRX;
import frc.robot.utilities.MotorConfig;

public class PIDFTalonSRX extends TrigonTalonSRX implements PIDFTalon {
    private PIDFCoefs pidfCoefs;
    private boolean isTuning;
    private double tuningSetpoint;

    /**
     * Constructs a new PIDF motor controller
     *
     * @param id          device ID of motor controller
     * @param motorConfig The configuration preset to use
     */
    public PIDFTalonSRX(int id, MotorConfig motorConfig) {
        super(id, motorConfig);

        setCoefs(motorConfig.getCoefs());
        this.isTuning = false;
        this.tuningSetpoint = 0;
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
    public void setPIDFCoefs(PIDFCoefs coefs) {
        pidfCoefs = coefs;
    }

    @Override
    public double getTuningSetpoint() {
        return tuningSetpoint;
    }

    @Override
    public void setSetpoint(double setpoint) {
        tuningSetpoint = setpoint;
    }

    @Override
    public boolean isTuning() {
        return isTuning;
    }

    public void initSendable(SendableBuilder builder) {
        PIDFTalon.super.initSendable(builder);
    }
}
