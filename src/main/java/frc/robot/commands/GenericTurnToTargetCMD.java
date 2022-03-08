package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.RobotConstants.VisionConstants;
import frc.robot.subsystems.MovableSubsystem;
import frc.robot.utilities.pid.TrigonPIDController;
import frc.robot.vision.Limelight;

/**
 * This is just template for a subsystem turn to target command. It will
 * probably be changed according the game and the robot.
 */
public class GenericTurnToTargetCMD extends CommandBase {
    private final Limelight limelight;
    private final MovableSubsystem subsystem;
    private final PIDController rotationPIDController;
    private double lastTimeSeenTarget;

    public GenericTurnToTargetCMD(Limelight limelight, MovableSubsystem subsystem) {

        addRequirements(subsystem);

        this.limelight = limelight;
        this.subsystem = subsystem;

        rotationPIDController = new TrigonPIDController(VisionConstants.ROTATION_SETTINGS);
        SmartDashboard.putData("turn to target coefs", rotationPIDController);
    }

    @Override
    public void initialize() {
        rotationPIDController.reset();
        rotationPIDController.setSetpoint(0);
        lastTimeSeenTarget = Timer.getFPGATimestamp();
        // Configure the limelight to start computing vision.
        limelight.startVision();
    }

    @Override
    public void execute() {
        if(limelight.getTv()) {
            subsystem.move(rotationPIDController.calculate(-limelight.getTx()));
            lastTimeSeenTarget = Timer.getFPGATimestamp();
        } else
            // The target wasn't found
            subsystem.stopMoving();
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.stopMoving();
    }

    @Override
    public boolean isFinished() {
        return ((Timer.getFPGATimestamp() - lastTimeSeenTarget) > VisionConstants.TARGET_TIME_OUT);
    }

    public boolean atSetpoint() {
        SmartDashboard.putBoolean("shoot/turn at setpoint", Math.abs(
                limelight.getTx() - rotationPIDController.getSetpoint()) < VisionConstants.ROTATION_SETTINGS.getTolerance());
        return Math.abs(
                limelight.getTx() - rotationPIDController.getSetpoint()) < VisionConstants.ROTATION_SETTINGS.getTolerance();
    }

    public void enableTuning() {
        SmartDashboard.putData("PID/visionRotation", rotationPIDController);
    }
}
