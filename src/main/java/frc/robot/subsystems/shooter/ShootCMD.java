package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.DoubleSupplier;

public class ShootCMD extends CommandBase {
    private final ShooterSS shooterSS;
    private final DoubleSupplier setpoint;
    private boolean wasAtSetpoint;

    private double ballsShot;
    private boolean isShootingBall;
    private static final double TOLERANCE = 0.1;

    public ShootCMD(ShooterSS shooterSS, DoubleSupplier setpoint) {
        this.shooterSS = shooterSS;
        this.setpoint = setpoint;

        addRequirements(shooterSS);
    }

    @Override
    public void initialize() {
        wasAtSetpoint = false;
        ballsShot = 0;
        isShootingBall = false;
    }

    @Override
    public void execute() {
        shooterSS.setSetpoint(setpoint.getAsDouble());

        wasAtSetpoint = wasAtSetpoint || (shooterSS.atSetpoint() && shooterSS.getSetpoint() > 0);
        if(!isShootingBall && wasAtSetpoint && shooterSS.getError() > setpoint.getAsDouble() * TOLERANCE) {
            isShootingBall = true;
        }
        if(isShootingBall && shooterSS.getError() < setpoint.getAsDouble() * TOLERANCE) {
            isShootingBall = false;
            ballsShot++;
        }
    }

    @Override
    public boolean isFinished() {
        return ballsShot >= 2 && !isShootingBall;
    }

    @Override
    public void end(boolean interrupted) {
        shooterSS.stopMoving();
    }

    public boolean atSetpoint() {
        return shooterSS.atSetpoint() && shooterSS.getSetpoint() > 0;
    }

    public double getBallsShot() {
        return ballsShot;
    }
}
