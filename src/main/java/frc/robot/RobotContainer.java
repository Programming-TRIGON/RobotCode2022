package frc.robot;

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
        driverXbox = new TrigonXboxController(DriverConstants.XBOX_PORT);

        initializeSubsystems();
        initializeCommands();
        bindCommands();
    }

    /**
     * Initializes all subsystems
     */
    private void initializeSubsystems() {
        swerveSS = new SwerveSS();
    }

    /**
     * initializes all commands
     */
    private void initializeCommands() {
        boolean squared = DriverConstants.SQUARED_CONTROLLER_DRIVING;
        driveWithXboxCMD = new SupplierDriveCMD(
                swerveSS,
                squared ? () -> Math.pow(driverXbox.getLeftX(), 2) : driverXbox::getLeftX,
                squared ? () -> Math.pow(driverXbox.getLeftY(), 2) : driverXbox::getLeftY,
                squared ? () -> Math.pow(driverXbox.getRightX(), 2) : driverXbox::getRightX,
                true);
    }

    private void bindCommands() {
        swerveSS.setDefaultCommand(driveWithXboxCMD);
        driverXbox.getYBtn().whenPressed(new InstantCommand(() -> swerveSS.resetGyro()));
    }

    public void periodic() {
        dashboardController.update();
    }
}
