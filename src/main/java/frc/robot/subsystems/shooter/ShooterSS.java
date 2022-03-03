package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CharacterizationConstants;
import frc.robot.constants.RobotConstants.ShooterConstants;
import frc.robot.subsystems.CharacterizableSubsystem;
import frc.robot.subsystems.PIDSubsystem;
import frc.robot.subsystems.TestableSubsystem;
import frc.robot.utilities.Conversions;
import frc.robot.utilities.pid.PIDFTalonSRX;

public class ShooterSS extends SubsystemBase implements PIDSubsystem, CharacterizableSubsystem {
    private final PIDFTalonSRX masterMotor;

    public ShooterSS() {
        masterMotor = ShooterConstants.LEFT_MOTOR;

        ShooterConstants.LEFT_MOTOR.follow(masterMotor);
        ShooterConstants.RIGHT_MOTOR.follow(masterMotor);

        putCharacterizeCMDInDashboard();
    }

    /**
     * @param setpoint the desired velocity in RPM
     */
    @Override
    public void setSetpoint(double setpoint) {
        masterMotor.setSetpoint(Conversions.RPMToFalcon(setpoint));
    }

    /**
     * @param power to be set to the motors
     */
    @Override
    public void move(double power) {
        masterMotor.set(power);
    }

    /**
     * @return the velocity of the motors in RPM
     */
    public double getVelocityRPM() {
        return Conversions.falconToRPM(masterMotor.getSelectedSensorVelocity());
    }

    /**
     * sets the feedforward for all the different components of the subsystem
     *
     * @param kV velocity gains
     * @param kS static gains
     */
    @Override
    public void updateFeedforward(double[] kV, double[] kS) {
        masterMotor.getCoefs().setKV(kV[0]);
        masterMotor.getCoefs().setKS(kS[0]);
    }

    @Override
    public CharacterizationConstants getCharacterizationConstants() {
        return ShooterConstants.CHARACTERIZATION_CONSTANTS;
    }

    /**
     * @return an array of the current encoder position
     */
    @Override
    public double[] getValues() {
        return new double[] {masterMotor.getSelectedSensorVelocity()};
    }

    @Override
    public String getName() {
        return "Shooter";
    }
}
