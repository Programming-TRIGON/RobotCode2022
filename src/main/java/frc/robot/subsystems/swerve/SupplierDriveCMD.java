package frc.robot.subsystems.swerve;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.RobotConstants.SwerveConstants;

import java.util.function.Supplier;

/**
 * A command that uses double suppliers (x, y, theta) to drive the robot, open loop.
 */
public class SupplierDriveCMD extends CommandBase {

    private final Supplier<Double> xPower;
    private final Supplier<Double> yPower;
    private final Supplier<Double> rotPower;
    private final boolean fieldRelative;
    private final SwerveSS swerveSS;

    public SupplierDriveCMD(
            SwerveSS swerveSS, Supplier<Double> xPower, Supplier<Double> yPower, Supplier<Double> rotPower,
            boolean fieldRelative) {
        this.swerveSS = swerveSS;
        this.xPower = xPower;
        this.yPower = yPower;
        this.rotPower = rotPower;
        this.fieldRelative = fieldRelative;
        addRequirements(swerveSS);
    }

    @Override
    public void execute() {
        swerveSS.drive(
                xPower.get() * SwerveConstants.MAX_SPEED,
                yPower.get() * SwerveConstants.MAX_SPEED,
                rotPower.get() * SwerveConstants.MAX_ANGULAR_VELOCITY,
                fieldRelative,
                true);
    }

    @Override
    public void end(boolean interrupted) {
        swerveSS.stopMoving();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
