package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ClimbCMD extends CommandBase {
    private final ClimberSS climberSS;
    private double pos;

    public ClimbCMD(ClimberSS climberSS) {
        this.climberSS = climberSS;

        addRequirements();
    }

    public void schedule(double pos) {
        this.pos = pos;
        super.schedule();
    }

    public void initialize() {
        climberSS.setSetpoint(pos);
    }

    @Override
    public void execute() {

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
