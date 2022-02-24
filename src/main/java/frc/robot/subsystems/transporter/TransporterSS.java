package frc.robot.subsystems.transporter;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorSensorV3.RawColor;
import edu.wpi.first.wpilibj.DriverStation;
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

    /**
     * Get the raw color values from their respective ADCs (20-bit).
     *
     * @return ColorValues struct containing red, green, blue and IR values
     */
    public RawColor getRawColor() {
        return colorSensor.getRawColor();
    }

    /**
     * Get the most likely color. Works best when within 2 inches and perpendicular to surface of
     * interest.
     *
     * @return Color enum of the most likely color, including unknown if the minimum threshold is not
     * met
     */
    public Color getColor() {
        return colorSensor.getColor();
    }

    /**
     * @return If the color sensor sees a ball belonging to the alliance.
     */
    public boolean seesAllianceBall() {
        Color allianceColor = DriverStation.getAlliance() == DriverStation.Alliance.Blue ?
                              Color.kFirstBlue :
                              Color.kFirstRed;
        return getColor().equals(allianceColor);
    }

    /**
     * @return If the color sensor sees a ball belonging to the rival alliance.
     */
    public boolean seesRivalBall() {
        Color allianceColor = DriverStation.getAlliance() == DriverStation.Alliance.Blue ?
                              Color.kFirstRed :
                              Color.kFirstBlue;
        return getColor().equals(allianceColor);
    }

    /**
     * @return If the color sensor sees any ball.
     */
    public boolean seesBall() {
        return getColor().equals(Color.kFirstBlue) || getColor().equals(Color.kFirstRed);
    }

    /**
     * Get the raw proximity value from the sensor ADC (11 bit). This value is largest when an object
     * is close to the sensor and smallest when far away.
     *
     * @return Proximity measurement value, ranging from 0 to 2047
     */
    public int getProximity() {
        return colorSensor.getProximity();
    }
}

