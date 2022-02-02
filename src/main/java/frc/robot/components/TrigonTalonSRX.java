package frc.robot.components;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.utilities.ConfigurableTalon;
import frc.robot.utilities.MotorConfig;

/**
 * This class creates a new instance of WPI_TalonSRX and configures values based
 * on a config preset
 */
public class TrigonTalonSRX extends WPI_TalonSRX implements ConfigurableTalon {

    /**
     * constructs a new motor controller
     *
     * @param id          device ID of motor controller
     * @param motorConfig The configuration preset to use
     */
    public TrigonTalonSRX(int id, MotorConfig motorConfig) {
        super(id);
        ce_configFactoryDefault();
        config(motorConfig);
    }

    /**
     * constructs a new motor controller with a default configuration
     *
     * @param id device ID of motor controller
     */
    public TrigonTalonSRX(int id) {
        this(id, new MotorConfig());
    }
}
