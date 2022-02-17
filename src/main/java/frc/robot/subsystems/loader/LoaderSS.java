package frc.robot.subsystems.loader;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.components.TrigonTalonSRX;
import frc.robot.constants.RobotConstants.LoaderConstants;
import frc.robot.subsystems.TestableSubsystem;
import frc.robot.utilities.pid.PIDFTalonFX;
import frc.robot.utilities.pid.PIDFTalonSRX;

public class LoaderSS extends SubsystemBase implements TestableSubsystem {
    // TODO: Set an ENABLE method, and make a motor instance.
    private final PIDFTalonSRX motor;

    public LoaderSS() {
        motor = LoaderConstants.MOTOR;
    }

    @Override
    public void move(double power) {
        motor.set(power);
    }

    public void setVelocity(double velocity){
        motor.setSetpoint();
    }

    public double getVelocity(){
        return motor.getSelectedSensorVelocity();
    }

    @Override
    public double[] getValues() {
        return new double[] {getVelocity()};
    }
}

