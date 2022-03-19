package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.GenericTurnToTargetCMD;
import frc.robot.commands.MoveMovableSubsystem;
import frc.robot.commands.RunWhenDisabledCommand;
import frc.robot.commands.commandgroups.ShootCG;
import frc.robot.components.TrigonXboxController;
import frc.robot.constants.RobotConstants;
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
import frc.robot.vision.CamMode;
import frc.robot.vision.Limelight;
import org.photonvision.PhotonCamera;

public class RobotContainer {
    private final DashboardController dashboardController;
    private final TrigonXboxController driverXbox;
    private final TrigonXboxController commanderXbox;
    public Limelight hubLimelight;
    public PhotonCamera cargoPhoton;

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
    private ShootCG shootCG;

    // States
    private boolean endgame = false;

    private final Button userBtn;

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
        hubLimelight = new Limelight("limelight-hub");
        hubLimelight.setCamMode(CamMode.vision);

        cargoPhoton = new PhotonCamera("photon-cargo");
        cargoPhoton.setPipelineIndex(DriverStation.getAlliance().equals(DriverStation.Alliance.Blue) ? 0 : 1);
        cargoPhoton.setDriverMode(false);

        userBtn = new Button(RobotController::getUserButton);

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
        shootCG = new ShootCG(this);
        SmartDashboard.putData("shootcgg", shootCG);
    }

    private void bindCommands() {
        swerveSS.setDefaultCommand(driveWithXboxCMD);

        driverXbox.getYBtn().whenPressed(new InstantCommand(swerveSS::resetGyro));
        driverXbox.getLeftBumperBtn().whenPressed(new InstantCommand(() -> intakeOpenerSS.setState(true)));
        driverXbox.getLeftBumperBtn().whileHeld(
                new ParallelCommandGroup(
                        new MoveMovableSubsystem(intakeSS, () -> IntakeConstants.POWER),
                        new MoveMovableSubsystem(transporterSS, () -> TransporterConstants.POWER)));
        driverXbox.getLeftBumperBtn().whenReleased(
                new ParallelCommandGroup(
                        new InstantCommand(() -> intakeOpenerSS.setState(false)),
                        new MoveMovableSubsystem(intakeSS, () -> IntakeConstants.POWER).withTimeout(0.4)
                                .andThen(new MoveMovableSubsystem(loaderSS, () -> -LoaderConstants.POWER).withTimeout(
                                        0.4)))
        );
        driverXbox.getBBtn().whileHeld(shootCG);
        driverXbox.getRightBumperBtn().whileHeld(new MoveMovableSubsystem(loaderSS, () -> LoaderConstants.POWER));
        driverXbox.getABtn()
                .whileHeld(new GenericTurnToTargetCMD(swerveSS, () -> -hubLimelight.getTx(), hubLimelight::getTv, 2,
                        RobotConstants.VisionConstants.HUB_TTT_COEFS,
                        1));

        commanderXbox.getBBtn().whileHeld(new ParallelCommandGroup(
                new MoveMovableSubsystem(intakeSS, () -> -IntakeConstants.POWER),
                new MoveMovableSubsystem(transporterSS, () -> -TransporterConstants.POWER),
                new MoveMovableSubsystem(loaderSS, () -> -LoaderConstants.POWER)));
        commanderXbox.getXBtn().whileHeld(new MoveMovableSubsystem(loaderSS, () -> -LoaderConstants.POWER));
        commanderXbox.getABtn().whenPressed(new InstantCommand(() -> setEndgame(!isEndgame())));

        //        userBtn.whenPressed(new RunWhenDisabledCommand(
        //                () -> hubLimelight.setLedMode(hubLimelight.getLedMode() == LedMode.off ? LedMode.on :
        //                LedMode.off)));
        userBtn.whenPressed(new RunWhenDisabledCommand(swerveSS::resetGyro));
    }

    /**
     * Puts the subsystems and commands in the dashboard
     */
    private void putData() {
        SmartDashboard.putData("Swerve", swerveSS);
        SmartDashboard.putData("Pitcher", pitcherSS);
        SmartDashboard.putData("Climber", climberSS);
        SmartDashboard.putData("Shooter", shooterSS);
        dashboardController.addNumber(
                "hubLimelight/distance", () -> ShootingCalculations.calculateDistance(hubLimelight.getTy()));
    }

    public void periodic() {
        CommandScheduler.getInstance().run();
        dashboardController.update();
        if(endgame) {
            runClimber();
        }
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
