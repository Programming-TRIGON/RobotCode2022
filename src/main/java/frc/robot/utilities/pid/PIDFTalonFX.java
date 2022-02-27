package frc.robot.utilities.pid;

import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.components.TrigonTalonFX;
import frc.robot.utilities.MotorConfig;

public class PIDFTalonFX extends TrigonTalonFX implements PIDFTalon {
    private final PIDFCoefs pidfCoefs;
    private boolean isTuning;

    /**
     * Constructs a new PIDF motor controller
     *
     * @param id          device ID of motor controller
     * @param motorConfig The configuration preset to use
     */
    public PIDFTalonFX(int id, MotorConfig motorConfig) {
        super(id, motorConfig);

        pidfCoefs = new PIDFCoefs(getRemoteCoefs());
        this.isTuning = false;

        setCoefs(getCoefs());
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
    public PIDFCoefs getRemoteCoefs() {
        return getConfig().getCoefs();
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
