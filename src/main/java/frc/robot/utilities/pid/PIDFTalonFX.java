package frc.robot.utilities.pid;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.components.TrigonTalonFX;
import frc.robot.utilities.MotorConfig;

public class PIDFTalonFX extends TrigonTalonFX implements PIDFTalon {
    private final ControlMode controlMode;
    private final PIDFCoefs pidfCoefs;
    private final PIDFCoefs remotePidfCoefs;
    private boolean isTuning;

    /**
     * Constructs a new PIDF motor controller
     *
     * @param id          device ID of motor controller
     * @param motorConfig The configuration preset to use
     */
    public PIDFTalonFX(int id, MotorConfig motorConfig, ControlMode controlMode) {
        super(id, motorConfig);

        remotePidfCoefs = motorConfig.getCoefs();
        pidfCoefs = new PIDFCoefs(remotePidfCoefs);
        this.isTuning = false;
        this.controlMode = controlMode;

        setCoefs(motorConfig.getCoefs());
    }

    @Override
    public ControlMode getControlMode() {
        return controlMode;
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
        return remotePidfCoefs;
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
