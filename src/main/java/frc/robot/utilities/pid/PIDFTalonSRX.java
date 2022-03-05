package frc.robot.utilities.pid;

import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.components.TrigonTalonSRX;
import frc.robot.utilities.MotorConfig;

public class PIDFTalonSRX extends TrigonTalonSRX implements PIDFTalon {
    private final MotorConfig motorConfig;
    private final PIDFCoefs pidfCoefs;
    private boolean isTuning;

    /**
     * Constructs a new PIDF motor controller
     *
     * @param id          device ID of motor controller
     * @param motorConfig The configuration preset to use
     */
    public PIDFTalonSRX(int id, MotorConfig motorConfig) {
        super(id, motorConfig);

        this.motorConfig = motorConfig;
        pidfCoefs = new PIDFCoefs(getRemoteCoefs());
        this.isTuning = false;

        setCoefs(getCoefs());
    }

    public MotorConfig getMotorConfig() {
        return motorConfig;
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
    public double getError() {
        return getClosedLoopError();
    }

    @Override
    public double getDeltaError() {
        return getErrorDerivative();
    }

    public PIDFCoefs getRemoteCoefs() {
        return getMotorConfig().getCoefs();
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
