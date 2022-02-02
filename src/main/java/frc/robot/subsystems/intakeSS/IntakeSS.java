package frc.robot.subsystems.intakeSS;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.pid.PIDFTalonSRX;

public class IntakeSS extends SubsystemBase {

    private PIDFTalonSRX Motor;
    private IntakeSS intakeSS;

    public IntakeSS(PIDFTalonSRX Motor) {
        this.Motor = Motor;

    }

    public void setPower(double Power){
        Motor.set(Power);

    }

}
