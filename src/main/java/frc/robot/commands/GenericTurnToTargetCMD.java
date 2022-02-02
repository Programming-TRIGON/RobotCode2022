package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.RobotConstants.VisionConstants;
import frc.robot.subsystems.MovableSubsystem;
import frc.robot.utilities.pid.PIDCoefs;
import frc.robot.vision.Limelight;
import frc.robot.vision.Target;

/**
 * This is just template for a subsystem turn to target command. It will
 * probably be changed according the game and the robot.
 */
public class GenericTurnToTargetCMD extends CommandBase {
    private final Limelight limelight;
    private final MovableSubsystem subsystem;
    private final Target target;
    private final PIDController rotationPIDController;
    private double lastTimeSeenTarget;

    public GenericTurnToTargetCMD(
            Limelight limelight, Target target,
            MovableSubsystem subsystem) {

        addRequirements(subsystem);

        this.limelight = limelight;
        this.target = target;
        this.subsystem = subsystem;

        PIDCoefs rotationSettings = VisionConstants.ROTATION_SETTINGS;
        rotationPIDController = new PIDController(rotationSettings.getKP(), rotationSettings.getKI(),
                rotationSettings.getKD());
        rotationPIDController.setTolerance(rotationSettings.getTolerance(), rotationSettings.getDeltaTolerance());
    }

    @Override
    public void initialize() {
        rotationPIDController.reset();
        rotationPIDController.setSetpoint(0);
        lastTimeSeenTarget = Timer.getFPGATimestamp();
        // Configure the limelight to start computing vision.
        limelight.startVision(target);
    }

    @Override
    public void execute() {
        if(limelight.getTv()) {
            subsystem.move(rotationPIDController.calculate(limelight.getAngle()));
            lastTimeSeenTarget = Timer.getFPGATimestamp();
        } else
            // The target wasn't found
            subsystem.stopMoving();
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.stopMoving();
        limelight.stopVision();
    }

    @Override
    public boolean isFinished() {
        return ((Timer.getFPGATimestamp() - lastTimeSeenTarget) > VisionConstants.TARGET_TIME_OUT)
                || rotationPIDController.atSetpoint();
    }

    public void enableTuning() {
        SmartDashboard.putData("PID/visionRotation", rotationPIDController);
    }
}
