package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.commands.commandgroups.auto.SimpleAutoCG;

public class Robot extends TimedRobot {
    private RobotContainer robotContainer;
    private SimpleAutoCG simpleAutoCG;

    @Override
    public void robotInit() {
        robotContainer = new RobotContainer();
        simpleAutoCG = new SimpleAutoCG(robotContainer);
    }

    @Override
    public void robotPeriodic() {
        robotContainer.periodic();
    }

    @Override
    public void autonomousInit() {
        simpleAutoCG.schedule();
        robotContainer.climberSS.resetEncoders();
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        simpleAutoCG.cancel();
        robotContainer.setEndgame(false);
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
