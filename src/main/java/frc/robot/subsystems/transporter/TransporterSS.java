package frc.robot.subsystems.transporter;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorSensorV3.RawColor;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.components.TrigonTalonSRX;
import frc.robot.constants.RobotConstants.TransporterConstants;
import frc.robot.subsystems.OverridableSubsystem;

public class TransporterSS extends OverridableSubsystem {
    private final TrigonTalonSRX motor;
    private final ColorSensorV3 colorSensor;
    public TransporterSS() {
        motor = TransporterConstants.MOTOR;
        colorSensor = TransporterConstants.COLOR_SENSOR;
    }

    @Override
    public void overriddenMove(double power) {
        motor.set(power);
    }

    /**
     * @return the current being given to the motor
     */
    public double getStatorCurrent() {
        return motor.getStatorCurrent();
    }

    /**
     * @return if the motor is currently stalled
     */
    public boolean isStalled() {
        return motor.getStatorCurrent() > TransporterConstants.STALL_CURRENT_LIMIT;
    }

    public RawColor getRawColor() {
        return colorSensor.getRawColor();
    }

    public Color getColor() {
        return colorSensor.getColor();
    }

    public int getProximity() {
        return colorSensor.getProximity();
    }
}

