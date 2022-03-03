package frc.robot.subsystems.loader;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotConstants.LoaderConstants;
import frc.robot.subsystems.PIDSubsystem;
import frc.robot.subsystems.TestableSubsystem;
import frc.robot.utilities.Conversions;
import frc.robot.utilities.pid.PIDFTalonSRX;

public class LoaderSS extends SubsystemBase implements TestableSubsystem, PIDSubsystem {
    private final PIDFTalonSRX motor;

    public LoaderSS() {
        motor = LoaderConstants.MOTOR;

        SmartDashboard.putData("Loader/motor", motor);
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

    /**
     * @return the current velocity in RPM
     */
    public double getVelocity() {
        return Conversions.falconToRPM(motor.getSelectedSensorVelocity());
    }

    @Override
    public double[] getValues() {
        return new double[] {getVelocity()};
    }
}

