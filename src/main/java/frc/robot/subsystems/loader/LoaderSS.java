package frc.robot.subsystems.loader;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CharacterizationConstants;
import frc.robot.constants.RobotConstants.LoaderConstants;
import frc.robot.subsystems.CharacterizableSubsystem;
import frc.robot.subsystems.PIDSubsystem;
import frc.robot.subsystems.TestableSubsystem;
import frc.robot.utilities.Conversions;
import frc.robot.utilities.pid.PIDFTalonSRX;

public class LoaderSS extends SubsystemBase implements PIDSubsystem, CharacterizableSubsystem {
    private final PIDFTalonSRX motor;

    public LoaderSS() {
        motor = LoaderConstants.MOTOR;

        putCharacterizeCMDInDashboard();
    }

    @Override
    public void move(double power) {
        motor.set(power);
    }

    /**
     * @param setpoint the desired velocity in RPM
     */
    @Override
    public void setSetpoint(double setpoint) {
        motor.setSetpoint(Conversions.RPMToFalcon(setpoint));
    }

    @Override
    public boolean atSetpoint() {
        return motor.atSetpoint();
    }

    /**
     * @return the current velocity in RPM
     */
    public double getVelocity() {
        return Conversions.falconToRPM(motor.getSelectedSensorVelocity());
    }

    /**
     * sets the feedforward for all the different components of the subsystem
     *
     * @param kV velocity gains
     * @param kS static gains
     */
    @Override
    public void updateFeedforward(double[] kV, double[] kS) {
        motor.getCoefs().setKV(kV[0]);
        motor.getCoefs().setKS(kS[0]);
    }

    @Override
    public CharacterizationConstants getCharacterizationConstants() {
        return LoaderConstants.CHARACTERIZATION_CONSTANTS;
    }

    @Override
    public double[] getValues() {
        return new double[] {motor.getSelectedSensorVelocity()};
    }

    @Override
    public String getName() {
        return "Loader";
    }
}

