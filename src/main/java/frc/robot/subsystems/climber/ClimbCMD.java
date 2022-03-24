package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.Supplier;

public class ClimbCMD extends CommandBase {
    private final ClimberSS climberSS;
    private int posLeft;
    private int posRight;
    private Supplier<Boolean> isEndgame;

    public ClimbCMD(ClimberSS climberSS, int poseLeft, int poseRight, Supplier<Boolean> isEndgame) {
        this.climberSS = climberSS;
        this.posLeft = poseLeft;
        this.posRight = poseRight;
        this.isEndgame = isEndgame;

        addRequirements();
    }

    public void schedule(double poseLeft, double poseRight) {
        super.schedule();
    }

    public void initialize() {
        if(isEndgame.get())
            climberSS.setSetpoint(posLeft, posRight);
    }

    @Override
    public boolean isFinished() {
        return climberSS.atSetpoint() || !isEndgame.get();
    }

    @Override
    public void end(boolean interrupted) {
        climberSS.stopMoving();
    }
}
