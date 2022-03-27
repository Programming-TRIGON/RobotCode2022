package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.RobotConstants;

import java.util.function.DoubleSupplier;

public class ShootCMD extends CommandBase {
    private final ShooterSS shooterSS;
    private DoubleSupplier setpoint;
    private boolean wasAtSetpoint;

    private boolean resetI = false;
    private double startTime;

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
        resetI = false;
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        if(Timer.getFPGATimestamp() - startTime > RobotConstants.ShooterConstants.I_WAIT_TIME) {
            if(!resetI) {
                shooterSS.resetI();
                shooterSS.setKi(shooterSS.getKI());
                resetI = true;
            }
        } else
            shooterSS.setKi(0);

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
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        shooterSS.stopMoving();
    }

    public boolean atSetpoint() {
        return shooterSS.atSetpoint() && shooterSS.getVelocity() > 2000;
    }

    public void setSetpoint(DoubleSupplier setpoint) {
        this.setpoint = setpoint;
    }

    public double getBallsShot() {
        return ballsShot;
    }
}
