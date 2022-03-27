package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.commandgroups.auto.*;

public class Robot extends TimedRobot {
    private RobotContainer robotContainer;
    private SendableChooser<Command> autoChooser;
    private SimpleAutoCG simpleAutoCG;

    @Override
    public void robotInit() {
        robotContainer = new RobotContainer();
        autoChooser = new SendableChooser<>();

        autoChooser.setDefaultOption("Backup Auto", new BackupAutoCG(robotContainer));
        autoChooser.addOption("Simple Auto", new SimpleAutoCG(robotContainer));
        autoChooser.addOption("Left Stealing Auto", new StealingAutoCG(robotContainer, () -> true));
        autoChooser.addOption("Right Stealing Auto", new StealingAutoCG(robotContainer, () -> false));
        autoChooser.addOption("Three Ball Normal Auto", new ThreeBallAutoCG(robotContainer));
        autoChooser.addOption("Four Ball Stealing Auto", new FourBallStealingAutoCG(robotContainer));

        SmartDashboard.putData(autoChooser);
    }

    @Override
    public void robotPeriodic() {
        robotContainer.periodic();
    }

    @Override
    public void autonomousInit() {
        robotContainer.setCargoLimelightPipeline();
        autoChooser.getSelected().schedule();
        robotContainer.climberSS.resetEncoders();
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        robotContainer.setCargoLimelightPipeline();
        autoChooser.getSelected().cancel();
        robotContainer.setEndgame(false);
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void disabledInit() {
        robotContainer.shooterSS.stopMoving();
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
