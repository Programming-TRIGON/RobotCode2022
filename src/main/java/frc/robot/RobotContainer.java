package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.GenericTurnToTargetCMD;
import frc.robot.commands.MoveMovableSubsystem;
import frc.robot.commands.commandgroups.ShootCG;
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
import frc.robot.subsystems.shooter.ShootingCalculations;
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
    public Limelight hubLimelight;
    public Limelight cargoLimelight;

    // Subsystems
    public SwerveSS swerveSS;
    public ShooterSS shooterSS;
    public PitcherSS pitcherSS;
    public LoaderSS loaderSS;
    public TransporterSS transporterSS;
    public ClimberSS climberSS;
    public LED ledSS;
    public IntakeSS intakeSS;
    public IntakeOpenerSS intakeOpenerSS;
    // Commands
    private SupplierDriveCMD driveWithXboxCMD;
    public MoveMovableSubsystem intakeCMD;
    public MoveMovableSubsystem transportCMD;
    private MoveMovableSubsystem inverseIntakeCMD;
    private MoveMovableSubsystem inverseTransportCMD;
    private ShootCG shootCG;

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
        hubLimelight = new Limelight("hub");
        hubLimelight.startVision();
        cargoLimelight = new Limelight("cargo");
        cargoLimelight.startVision();

        HttpCamera limelightFeed = new HttpCamera(
                "cargo", "http://limelight-cargo:5800/stream.mjpg", HttpCamera.HttpCameraKind.kMJPGStreamer);
        CameraServer.startAutomaticCapture(limelightFeed);

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
                         (intakeSS.isCollecting() ?
                          DriverConstants.COLLECT_SPEED_DIVIDER :
                          DriverConstants.SPEED_DIVIDER)),
                () -> driverXbox.getLeftY() /
                        (isEndgame() ?
                         DriverConstants.ENDGAME_SPEED_DIVIDER :
                         (intakeSS.isCollecting() ?
                          DriverConstants.COLLECT_SPEED_DIVIDER :
                          DriverConstants.SPEED_DIVIDER)),
                () -> driverXbox.getRightX() /
                        (isEndgame() ?
                         DriverConstants.ENDGAME_ROTATION_SPEED_DIVIDER :
                         (intakeSS.isCollecting() ?
                          DriverConstants.COLLECT_ROTATION_SPEED_DIVIDER :
                          DriverConstants.ROTATION_SPEED_DIVIDER)),
                true);
        intakeCMD = new MoveMovableSubsystem(intakeSS, () -> IntakeConstants.POWER);
        transportCMD = new MoveMovableSubsystem(transporterSS, () -> TransporterConstants.POWER);
        inverseIntakeCMD = new MoveMovableSubsystem(intakeSS, () -> -IntakeConstants.POWER);
        inverseTransportCMD = new MoveMovableSubsystem(transporterSS, () -> -TransporterConstants.POWER);
        shootCG = new ShootCG(this, () -> ShootingCalculations.calculateVelocity(hubLimelight.getDistance()), false);
    }

    private void bindCommands() {
        swerveSS.setDefaultCommand(driveWithXboxCMD);

        driverXbox.getYBtn().whenPressed(new InstantCommand(swerveSS::resetGyro));
        driverXbox.getLeftBumperBtn().whileHeld(
                new SequentialCommandGroup(
                        new InstantCommand(() -> intakeOpenerSS.setState(true)),
                        new ParallelCommandGroup(intakeCMD, transportCMD)));
        driverXbox.getLeftBumperBtn().whenReleased(
                new ParallelCommandGroup(
                        new InstantCommand(() -> intakeOpenerSS.setState(false)),
                        new MoveMovableSubsystem(intakeSS, () -> IntakeConstants.POWER).withTimeout(0.4)
                                .andThen(new MoveMovableSubsystem(loaderSS, () -> 0.7).withTimeout(0.4)))
        );
        driverXbox.getXBtn().whileHeld(
                new GenericTurnToTargetCMD(
                        hubLimelight, swerveSS, true));
        driverXbox.getBBtn().whileHeld(
                new ShootCG(this, () -> ShootingCalculations.calculateVelocity(hubLimelight.getDistance()), false));
        ;

        commanderXbox.getBBtn().whileHeld(new ParallelCommandGroup(inverseIntakeCMD, inverseTransportCMD,
                new MoveMovableSubsystem(loaderSS, () -> 0.7)));
        //        commanderXbox.getXBtn().whileHeld(new LoaderCMD)
        //        climbCMD.withInterrupt(commanderXbox::getXButton);
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
        SmartDashboard.putData("Shooter", shooterSS
        );
        //        SmartDashboard.putData("Loader/move", new MoveMovableSubsystem(loaderSS, () -> LoaderConstants
        //        .POWER));
        SmartDashboard.putNumber("ShootCGa/velocity", 0);
        SmartDashboard.putData("ShootCGa/ShootCGa", new ShootCG(this,
                () -> SmartDashboard.getNumber("ShootCGa/velocity", 0), true));
        dashboardController.addNumber("limelight/distance", () -> hubLimelight.getDistance());
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
            hubLimelight.setLedMode(hubLimelight.getLedMode() == LedMode.off ? LedMode.on : LedMode.off);
        }
        //        if(driverXbox.getBButton())
        //            loaderSS.move(LoaderConstants.POWER);
        //        else
        //            loaderSS.stopMoving();
    }

    private void runClimber() {
        if(driverXbox.getRightTriggerAxis() > ClimberConstants.TRIGGER_DEADBAND)
            new ClimbCMD(climberSS, ClimberConstants.MAX_LEFT_POSE, ClimberConstants.MAX__RIGHT_POSITION).schedule();
        else if(driverXbox.getLeftTriggerAxis() > ClimberConstants.TRIGGER_DEADBAND)
            new ClimbCMD(climberSS, -ClimberConstants.MAX_LEFT_POSE, -ClimberConstants.MAX__RIGHT_POSITION).schedule();
        else if(commanderXbox.getPOV() == 270)
            new ClimbCMD(climberSS, 0, 0).schedule();

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
        if(!commanderXbox.getRightBumper())
            climberSS.moveRight(0);
        if(!commanderXbox.getLeftBumper())
            climberSS.moveLeft(0);
    }

    public boolean isEndgame() {
        return endgame;
    }

    public void setEndgame(boolean endgame) {
        this.endgame = endgame;
    }
}
