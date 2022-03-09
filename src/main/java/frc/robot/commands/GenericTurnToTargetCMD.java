package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.RobotConstants.VisionConstants;
import frc.robot.subsystems.swerve.SwerveSS;
import frc.robot.utilities.pid.TrigonPIDController;
import frc.robot.vision.Limelight;

/**
 * This is just template for a subsystem turn to target command. It will
 * probably be changed according the game and the robot.
 */
public class GenericTurnToTargetCMD extends CommandBase {
    private final Limelight limelight;
    private final SwerveSS swerveSS;
    private final PIDController rotationPIDController;
    private final PIDController distancePIDController;
    private double lastTimeSeenTarget;
    private boolean driveToTarget;

    public GenericTurnToTargetCMD(Limelight limelight, SwerveSS swerveSS, boolean driveToTarget) {

        addRequirements(swerveSS);

        this.limelight = limelight;
        this.swerveSS = swerveSS;
        this.driveToTarget = driveToTarget;

        rotationPIDController = new TrigonPIDController(VisionConstants.ROTATION_SETTINGS);
        distancePIDController = new TrigonPIDController(VisionConstants.DRIVE_SETTINGS);
        SmartDashboard.putData("turn to target coefs", rotationPIDController);
        SmartDashboard.putData("drive to target coefs", distancePIDController);
    }

    @Override
    public void initialize() {
        rotationPIDController.reset();
        rotationPIDController.setSetpoint(1.5);
        distancePIDController.reset();
        distancePIDController.setSetpoint(2);
        lastTimeSeenTarget = Timer.getFPGATimestamp();
        // Configure the limelight to start computing vision.
        limelight.startVision();
    }

    @Override
    public void execute() {
        if(limelight.getTv()) {
            if(driveToTarget)
                swerveSS.drive(0, distancePIDController.calculate(limelight.getDistance()),
                        rotationPIDController.calculate(-limelight.getTx()), false, false);
            else
                swerveSS.move(rotationPIDController.calculate(-limelight.getTx()));
            lastTimeSeenTarget = Timer.getFPGATimestamp();
        } else
            // The target wasn't found
            swerveSS.stopMoving();
    }

    @Override
    public void end(boolean interrupted) {
        swerveSS.stopMoving();
    }

    @Override
    public boolean isFinished() {
        return ((Timer.getFPGATimestamp() - lastTimeSeenTarget) > VisionConstants.TARGET_TIME_OUT);
    }

    public boolean atSetpoint() {
        SmartDashboard.putBoolean("shoot/turn at setpoint", Math.abs(
                limelight.getTx() - rotationPIDController.getSetpoint()) < VisionConstants.ROTATION_SETTINGS.getTolerance());
        return distancePIDController.atSetpoint();
    }

    public void enableTuning() {
        SmartDashboard.putData("PID/visionRotation", rotationPIDController);
    }
}
