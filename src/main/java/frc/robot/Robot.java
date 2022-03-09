package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.commands.commandgroups.BackupAutoCG;

public class Robot extends TimedRobot {
    private RobotContainer robotContainer;
    private BackupAutoCG backupAutoCG;

    @Override
    public void robotInit() {
        robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        robotContainer.periodic();
    }

    @Override
    public void autonomousInit() {
        backupAutoCG = new BackupAutoCG(robotContainer);
        backupAutoCG.schedule();
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        backupAutoCG.cancel();
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void testInit() {
    }

    @Override
    public void testPeriodic() {
    }
}
