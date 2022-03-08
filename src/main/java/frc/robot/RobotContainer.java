package frc.robot;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.MoveMovableSubsystem;
import frc.robot.components.TrigonXboxController;
import frc.robot.constants.RobotConstants.*;
import frc.robot.subsystems.climber.ClimbCMD;
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
import frc.robot.vision.LedMode;
import frc.robot.vision.Limelight;

public class RobotContainer {
    private final DashboardController dashboardController;
    private final TrigonXboxController driverXbox;
    private final TrigonXboxController commanderXbox;
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
    private MoveMovableSubsystem inverseIntakeCMD;
    private MoveMovableSubsystem inverseTransportCMD;
    private ClimbCMD climbCMD;

    // States
    private boolean endgame = false;
    private boolean userBtnPressed;

    /**
     * Add classes here
     */
    public RobotContainer() {
        dashboardController = new DashboardController();
        driverXbox = new TrigonXboxController(
                DriverConstants.XBOX_PORT,
                DriverConstants.CONTROLLER_DEADBAND,
                DriverConstants.SQUARED_CONTROLLER_DRIVING);
        commanderXbox = new TrigonXboxController(
                CommanderConstants.XBOX_PORT,
                CommanderConstants.CONTROLLER_DEADBAND,
                CommanderConstants.SQUARED_CONTROLLER_DRIVING);
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
                () -> driverXbox.getLeftX() /
                        (isEndgame() ?
                         DriverConstants.ENDGAME_SPEED_DIVIDER :
                         DriverConstants.SPEED_DIVIDER),
                () -> driverXbox.getLeftY() /
                        (isEndgame() ?
                         DriverConstants.ENDGAME_SPEED_DIVIDER :
                         DriverConstants.SPEED_DIVIDER),
                () -> driverXbox.getRightX() /
                        (isEndgame() ?
                         DriverConstants.ENDGAME_ROTATION_SPEED_DIVIDER :
                         DriverConstants.ROTATION_SPEED_DIVIDER),
                true);
        intakeCMD = new MoveMovableSubsystem(intakeSS, () -> IntakeConstants.POWER);
        transportCMD = new MoveMovableSubsystem(transporterSS, () -> TransporterConstants.POWER);
        inverseIntakeCMD = new MoveMovableSubsystem(intakeSS, () -> -IntakeConstants.POWER);
        inverseTransportCMD = new MoveMovableSubsystem(transporterSS, () -> -TransporterConstants.POWER);
        climbCMD = new ClimbCMD(climberSS);
    }

    private void bindCommands() {
        swerveSS.setDefaultCommand(driveWithXboxCMD);

        driverXbox.getYBtn().whenPressed(new InstantCommand(swerveSS::resetGyro));
        driverXbox.getLeftBumperBtn().whenPressed(new InstantCommand(intakeOpenerSS::toggleState));
        driverXbox.getLeftBumperBtn().whileHeld(new ParallelCommandGroup(inverseIntakeCMD, inverseTransportCMD));

        climbCMD.withInterrupt(commanderXbox::getXButton);
        commanderXbox.getABtn().whenPressed(new InstantCommand(() -> {
            setEndgame(!isEndgame());
        }));
    }

    /**
     * Puts the subsystems and commands in the dashboard
     */
    private void putData() {
        SmartDashboard.putData("Swerve", swerveSS);
        SmartDashboard.putData("Pitcher", pitcherSS);
        SmartDashboard.putData("Climber", climberSS);
    }

    public void periodic() {
        CommandScheduler.getInstance().run();
        dashboardController.update();
        if(endgame) {
            runClimber();
        }
        if(userBtnPressed) {
            if(!RobotController.getUserButton())
                userBtnPressed = false;
        } else if(RobotController.getUserButton()) {
            userBtnPressed = true;
            limelight.setLedMode(limelight.getLedMode() == LedMode.off ? LedMode.on : LedMode.off);
        }
    }

    private void runClimber() {
        if(driverXbox.getRightTriggerAxis() > ClimberConstants.TRIGGER_DEADBAND)
            climbCMD.schedule(ClimberConstants.MAX_POSITION);
        else if(driverXbox.getLeftTriggerAxis() > ClimberConstants.TRIGGER_DEADBAND)
            climbCMD.schedule(ClimberConstants.MIN_POSITION);
        else if(commanderXbox.getPOV() == 270)
            climbCMD.schedule(0);

        if(commanderXbox.getYButton()) {
            if(commanderXbox.getRightBumper())
                climberSS.moveRight(-ClimberConstants.OVERRIDDEN_POWER);
            if(commanderXbox.getLeftBumper())
                climberSS.moveLeft(-ClimberConstants.OVERRIDDEN_POWER);
        } else {
            if(commanderXbox.getRightBumper())
                climberSS.moveRight(ClimberConstants.OVERRIDDEN_POWER);
            if(commanderXbox.getLeftBumper())
                climberSS.moveLeft(ClimberConstants.OVERRIDDEN_POWER);
        }
    }

    public boolean isEndgame() {
        return endgame;
    }

    public void setEndgame(boolean endgame) {
        this.endgame = endgame;
    }
}
