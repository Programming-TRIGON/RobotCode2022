package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

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
        swerveSS.drive(new Translation2d(xPower.get(), yPower.get()), rotPower.get(), fieldRelative, false);
    }

    @Override
    public void end(boolean interrupted) {
        swerveSS.drive(new Translation2d(0, 0), 0, false, false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
