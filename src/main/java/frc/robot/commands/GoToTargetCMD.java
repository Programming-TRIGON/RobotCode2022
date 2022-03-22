package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.SwerveSS;
import frc.robot.utilities.pid.PIDCoefs;
import frc.robot.utilities.pid.TrigonPIDController;
import frc.robot.vision.Limelight;

import java.util.function.DoubleSupplier;

public class GoToTargetCMD extends CommandBase {
    SwerveSS swerveSS;
    Limelight limelight;
    TrigonPIDController rotationPIDController;
    private double lastTimeSeenTarget;
    private final DoubleSupplier speed;
    private final double TIMEOUT;
    private boolean seenTarget;
    private boolean running;

    public GoToTargetCMD(
            SwerveSS swerveSS, Limelight limelight, PIDCoefs rotationCoefs, DoubleSupplier speed, double timeout) {
        this.swerveSS = swerveSS;
        this.limelight = limelight;
        this.rotationPIDController = new TrigonPIDController(rotationCoefs);
        this.speed = speed;
        this.TIMEOUT = timeout;

        addRequirements(swerveSS);
    }

    @Override
    public void initialize() {
        lastTimeSeenTarget = Timer.getFPGATimestamp();
        rotationPIDController.reset();
        seenTarget = false;
        running = true;
    }

    @Override
    public void execute() {
        if(limelight.getTv()) {
            seenTarget = true;
            swerveSS.drive(
                    0, getSpeed(rotationPIDController.getError()), rotationPIDController.calculate(-limelight.getTx()),
                    false, true);
            lastTimeSeenTarget = Timer.getFPGATimestamp();
        } else {
            swerveSS.drive(0, getSpeed(0), 0, false, true);
        }
        SmartDashboard.putBoolean("gtt/seen", seenTarget);
        SmartDashboard.putNumber("gtt/p", rotationPIDController.getP());
        SmartDashboard.putNumber("gtt/error", rotationPIDController.getError());
        SmartDashboard.putNumber("gtt/speed", getSpeed(rotationPIDController.getError()));
    }

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() - lastTimeSeenTarget > TIMEOUT;
    }

    @Override
    public void end(boolean interrupted) {
        swerveSS.stopMoving();
        running = false;
    }

    private double getSpeed(double error) {
        if(error == 0)
            return seenTarget ? speed.getAsDouble() : 0;
        return speed.getAsDouble();
    }

    public void putOnShuffleboard(String entry) {
        SmartDashboard.putData(entry + "/PID", rotationPIDController);
        SmartDashboard.putBoolean(entry + "/running", running);
    }
}
