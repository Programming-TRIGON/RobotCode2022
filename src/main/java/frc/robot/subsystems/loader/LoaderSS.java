package frc.robot.subsystems.loader;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.components.TrigonTalonSRX;
import frc.robot.constants.CharacterizationConstants;
import frc.robot.constants.RobotConstants.LoaderConstants;
import frc.robot.subsystems.CharacterizableSubsystem;
import frc.robot.subsystems.TestableSubsystem;
import frc.robot.utilities.pid.PIDFTalonFX;
import frc.robot.utilities.pid.PIDFTalonSRX;

public class LoaderSS extends SubsystemBase implements CharacterizableSubsystem {
    private final PIDFTalonSRX motor;

    public LoaderSS() {
        motor = LoaderConstants.MOTOR;
    }

    @Override
    public void move(double power) {
        motor.set(power);
    }

    public void setVelocity(double velocity) {
        motor.setSetpoint(velocity);
    }

    public double getVelocity() {
        return motor.getSelectedSensorVelocity();
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
}

