package frc.robot.components;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import frc.robot.constants.RobotConstants;
import frc.robot.utilities.CTREUtil;
import frc.robot.utilities.ConfigurableTalon;
import frc.robot.utilities.MotorConfig;

/**
 * This class creates a new instance of WPI_TalonFX and configures values based
 * on a config preset
 */
public class TrigonTalonFX extends WPI_TalonFX implements ConfigurableTalon {
    private final MotorConfig config;

    /**
     * constructs a new motor controller
     *
     * @param id          device ID of motor controller
     * @param motorConfig The configuration preset to use
     */
    public TrigonTalonFX(int id, MotorConfig motorConfig) {
        super(id);
        this.config = motorConfig;
        ce_configFactoryDefault();
        config(motorConfig);
    }

    /**
     * constructs a new motor controller with a default configuration
     *
     * @param id device ID of motor controller
     */
    public TrigonTalonFX(int id) {
        this(id, new MotorConfig());
    }

    @Override
    public MotorConfig getConfig() {
        return config;
    }

    public ErrorCode ce_configStatorCurrentLimit(StatorCurrentLimitConfiguration statorCurrentLimit) {
        return CTREUtil.checkError(
                () -> configStatorCurrentLimit(statorCurrentLimit, RobotConstants.DEFAULT_CAN_TIMEOUT));
    }
}
