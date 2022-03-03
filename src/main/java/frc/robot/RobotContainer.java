package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.components.TrigonXboxController;
import frc.robot.constants.RobotConstants.DriverConstants;
import frc.robot.subsystems.climber.ClimberSS;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.loader.LoaderSS;
import frc.robot.subsystems.pitcher.PitcherSS;
import frc.robot.subsystems.intake.IntakeOpenerSS;
import frc.robot.subsystems.intake.IntakeSS;
import frc.robot.subsystems.swerve.SupplierDriveCMD;
import frc.robot.subsystems.swerve.SwerveSS;
import frc.robot.subsystems.shooter.ShooterSS;
import frc.robot.subsystems.transporter.TransporterSS;
import frc.robot.utilities.DashboardController;

public class RobotContainer {
    private final DashboardController dashboardController;
    private final TrigonXboxController driverXbox;

    // Subsystems
    private SwerveSS swerveSS;
    private ShooterSS shooterSS;
    private IntakeSS intake;
    private IntakeOpenerSS intakeOpener;
    private ClimberSS climberSS;
    private LoaderSS loaderSS;
    private PitcherSS pitcherSS;
    private TransporterSS transporterSS;
    private LED ledSS;

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
        putData();
    }

    /**
     * Initializes all subsystems
     */
    private void initializeSubsystems() {
        swerveSS = new SwerveSS();
        shooterSS = new ShooterSS();
        intake = new IntakeSS();
        intakeOpener = new IntakeOpenerSS();
        climberSS = new ClimberSS();
        loaderSS = new LoaderSS();
        pitcherSS = new PitcherSS();
        transporterSS = new TransporterSS();
        ledSS = new LED();
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
    }

    private void bindCommands() {
        swerveSS.setDefaultCommand(driveWithXboxCMD);
        driverXbox.getYBtn().whenPressed(new InstantCommand(() -> swerveSS.resetGyro()));
    }

    /**
     * Puts the subsystems and commands in the dashboard
     */
    private void putData() {
        SmartDashboard.putData("Swerve", swerveSS);
    }

    public void periodic() {
        CommandScheduler.getInstance().run();
        dashboardController.update();
    }
}
