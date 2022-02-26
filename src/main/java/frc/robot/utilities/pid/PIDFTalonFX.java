package frc.robot.utilities.pid;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.components.TrigonTalonFX;
import frc.robot.utilities.MotorConfig;

public class PIDFTalonFX extends TrigonTalonFX implements PIDFTalon {
    private final ControlMode closedLoopControlMode;
    private final PIDFCoefs pidfCoefs;
    private final PIDFCoefs remotePidfCoefs;
    private boolean isTuning;

    /**
     * Constructs a new PIDF motor controller
     *
     * @param id                    device ID of motor controller
     * @param motorConfig           The configuration preset to use
     * @param closedLoopControlMode The control mode to use for closed loop control
     */
    public PIDFTalonFX(int id, MotorConfig motorConfig, ControlMode closedLoopControlMode) {
        super(id, motorConfig);

        remotePidfCoefs = motorConfig.getCoefs();
        pidfCoefs = new PIDFCoefs(getRemoteCoefs());
        this.isTuning = false;
        this.closedLoopControlMode = closedLoopControlMode;

        setCoefs(getCoefs());
    }

    public ControlMode getClosedLoopControlMode() {
        return closedLoopControlMode;
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
