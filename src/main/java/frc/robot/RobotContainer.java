package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.components.TrigonXboxController;
import frc.robot.constants.RobotConstants.DriverConstants;
import frc.robot.subsystems.swerve.SupplierDriveCMD;
import frc.robot.subsystems.swerve.SwerveSS;
import frc.robot.utilities.DashboardController;

public class RobotContainer {
    private final DashboardController dashboardController;
    private final TrigonXboxController driverXbox;

    // Subsystems
    private SwerveSS swerveSS;

    // Commands
    private SupplierDriveCMD driveWithXboxCMD;

    /**
     * Add classes here
     */
    public RobotContainer() {
        dashboardController = new DashboardController();
        driverXbox = new TrigonXboxController(
                DriverConstants.XBOX_PORT,
                DriverConstants.CONTROLLER_DEADBAND,
                DriverConstants.SQUARED_CONTROLLER_DRIVING);

        initializeSubsystems();
        initializeCommands();
        bindCommands();
    }

    /**
     * Initializes all subsystems
     */
    private void initializeSubsystems() {
        swerveSS = new SwerveSS();
        SmartDashboard.putData("Swerve", swerveSS);
    }

    /**
     * initializes all commands
     */
    private void initializeCommands() {
        driveWithXboxCMD = new SupplierDriveCMD(
                swerveSS,
                driverXbox::getLeftX,
                driverXbox::getLeftY,
                driverXbox::getRightX,
                true);
        SmartDashboard.putData("Drive CMD", driveWithXboxCMD);
    }

    private void bindCommands() {
        swerveSS.setDefaultCommand(driveWithXboxCMD);
        driverXbox.getYBtn().whenPressed(new InstantCommand(() -> swerveSS.resetGyro()));
    }

    public void periodic() {
        CommandScheduler.getInstance().run();
        dashboardController.update();
    }
}
