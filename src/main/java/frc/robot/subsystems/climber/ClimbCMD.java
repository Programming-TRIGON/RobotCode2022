package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ClimbCMD extends CommandBase {
    private final ClimberSS climberSS;
    private final int leftPosition;
    private final int rightPosition;

    public ClimbCMD(ClimberSS climberSS, int leftPosition, int rightPosition) {
        this.climberSS = climberSS;
        this.leftPosition = leftPosition;
        this.rightPosition = rightPosition;

        addRequirements(climberSS);
    }

    public void schedule(double poseLeft, double poseRight) {
        super.schedule();
    }

    public void initialize() {
        climberSS.setSetpoint(leftPosition, rightPosition);
    }

    @Override
    public boolean isFinished() {
        return climberSS.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        climberSS.stopMoving();
    }
}
