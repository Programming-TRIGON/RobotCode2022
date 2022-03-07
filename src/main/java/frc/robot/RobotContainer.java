package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.MoveMovableSubsystem;
import frc.robot.components.TrigonXboxController;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.RobotConstants.DriverConstants;
import frc.robot.subsystems.climber.ClimberSS;
import frc.robot.subsystems.intake.IntakeOpenerSS;
import frc.robot.subsystems.intake.IntakeSS;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.loader.LoaderSS;
import frc.robot.subsystems.shooter.PitcherSS;
import frc.robot.subsystems.shooter.ShooterSS;
import frc.robot.subsystems.swerve.SupplierDriveCMD;
import frc.robot.subsystems.swerve.SwerveSS;
import frc.robot.subsystems.transporter.TransporterSS;
import frc.robot.utilities.DashboardController;
import frc.robot.vision.Limelight;

public class RobotContainer {
    private final DashboardController dashboardController;
    private final TrigonXboxController driverXbox;
    public Limelight limelight;

    // Subsystems
    public SwerveSS swerveSS;
    public ShooterSS shooterSS;
    public PitcherSS pitcherSS;
    public LoaderSS loaderSS;
    public TransporterSS transporterSS;
    public ClimberSS climberSS;
    public LED ledSS;
    private IntakeSS intakeSS;
    private IntakeOpenerSS intakeOpenerSS;
    // Commands
    private SupplierDriveCMD driveWithXboxCMD;
    private MoveMovableSubsystem intakeCMD;
    private MoveMovableSubsystem transportCMD;

    /**
     * Add classes here
     */
    public RobotContainer() {
        dashboardController = new DashboardController();
        driverXbox = new TrigonXboxController(
                DriverConstants.XBOX_PORT,
                DriverConstants.CONTROLLER_DEADBAND,
                DriverConstants.SQUARED_CONTROLLER_DRIVING);
        limelight = new Limelight();

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
        pitcherSS = new PitcherSS();
        loaderSS = new LoaderSS();
        transporterSS = new TransporterSS();
        climberSS = new ClimberSS();
        intakeSS = new IntakeSS();
        intakeOpenerSS = new IntakeOpenerSS();
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
        intakeCMD = new MoveMovableSubsystem(intakeSS, () -> RobotConstants.IntakeConstants.POWER);
        transportCMD = new MoveMovableSubsystem(transporterSS, () -> RobotConstants.TransporterConstants.POWER);
    }

    private void bindCommands() {
        swerveSS.setDefaultCommand(driveWithXboxCMD);
        driverXbox.getYBtn().whenPressed(new InstantCommand(swerveSS::resetGyro));
        driverXbox.getRightBumperBtn().whileHeld(new ParallelCommandGroup(intakeCMD, transportCMD));
        driverXbox.getLeftBumperBtn().whenPressed(new InstantCommand(intakeOpenerSS::toggleState));
    }

    /**
     * Puts the subsystems and commands in the dashboard
     */
    private void putData() {
        SmartDashboard.putData("Swerve", swerveSS);
        SmartDashboard.putData("Pitcher", pitcherSS);
    }

    public void periodic() {
        CommandScheduler.getInstance().run();
        dashboardController.update();
    }
}
