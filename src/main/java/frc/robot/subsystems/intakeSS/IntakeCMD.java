package frc.robot.subsystems.intakeSS;

import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.DoubleSupplier;

public class IntakeCMD extends CommandBase {

    private IntakeSS intakeSS;
    private DoubleSupplier topMotorPower;


    public IntakeCMD(IntakeSS intakeSS, DoubleSupplier topMotorPower) {
        this.intakeSS = intakeSS;
        this.topMotorPower = topMotorPower;
        // each subsystem used by the command must be passed into the addRequirements() method (which takes a vararg
        // of Subsystem)
    }

    @Override
    public void initialize() {
        addRequirements(intakeSS);
    }

    @Override
    public void execute() {
        intakeSS.setPower(topMotorPower.getAsDouble());
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        intakeSS.setPower(0);
    }
}
