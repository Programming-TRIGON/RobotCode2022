package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ClimbCMD extends CommandBase {
    private final ClimberSS climberSS;
    private int posLEft;
    private int posRight;

    public ClimbCMD(ClimberSS climberSS, int poseLeft, int poseRight) {
        this.climberSS = climberSS;
        this.posLEft = poseLeft;
        this.posRight = poseRight;

        addRequirements();
    }

    public void schedule(double poseLeft, double poseRight) {
        super.schedule();
    }

    public void initialize() {
        climberSS.setSetpoint(posLEft, posRight);
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
