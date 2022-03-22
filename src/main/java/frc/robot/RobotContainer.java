package frc.robot;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.MoveMovableSubsystem;
import frc.robot.commands.RunWhenDisabledCommand;
import frc.robot.commands.TurnToTargetCMD;
import frc.robot.commands.commandgroups.IntakeCG;
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
import frc.robot.vision.CamMode;
import frc.robot.vision.Limelight;

public class RobotContainer {
    private final DashboardController dashboardController;
    private final TrigonXboxController driverXbox;
    private final TrigonXboxController operatorXbox;
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
        operatorXbox = new TrigonXboxController(
                CommanderConstants.XBOX_PORT,
                CommanderConstants.CONTROLLER_DEADBAND,
                CommanderConstants.SQUARED_CONTROLLER_DRIVING);

        cargoLimelight = new Limelight("limelight-cargo");
        //cargoLimelight.setPipeline(DriverStation.getAlliance().equals(DriverStation.Alliance.Blue) ? 0 : 1);
        cargoLimelight.setCamMode(CamMode.vision);
        hubLimelight = new Limelight("limelight-hub");
        hubLimelight.setCamMode(CamMode.vision);

        userBtn = new Button(RobotController::getUserButton);

        initializeSubsystems();
        initializeCommands();
        bindDriverCommands();
        bindOperatorCommands();

        //        userBtn.whenPressed(new RunWhenDisabledCommand(
        //                () -> hubLimelight.setLedMode(hubLimelight.getLedMode() == LedMode.off ? LedMode.on :
        //                LedMode.off)));
        userBtn.whenPressed(new RunWhenDisabledCommand(swerveSS::resetGyro));

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
    }

    private void bindDriverCommands() {
        swerveSS.setDefaultCommand(driveWithXboxCMD);
        
        driverXbox.getABtn()
                .whileHeld(new TurnToTargetCMD(swerveSS, () -> -hubLimelight.getTx(), hubLimelight::getTv,
                        () -> -1.5 / ShootingCalculations.calculateDistance(hubLimelight.getTy()),
                        VisionConstants.HUB_TURN_TO_TARGET_COEFS,
                        1));
        driverXbox.getXBtn().whileHeld(new IntakeCG(this));
        driverXbox.getBBtn().whileHeld(shootCG);
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
        driverXbox.getRightBumperBtn().whileHeld(new MoveMovableSubsystem(loaderSS, () -> LoaderConstants.POWER));
    }

    private void bindOperatorCommands() {
        operatorXbox.getABtn().whenPressed(new InstantCommand(() -> setEndgame(!isEndgame())));
        operatorXbox.getBBtn().whileHeld(new ParallelCommandGroup(
                new MoveMovableSubsystem(intakeSS, () -> -IntakeConstants.POWER),
                new MoveMovableSubsystem(transporterSS, () -> -TransporterConstants.POWER),
                new MoveMovableSubsystem(loaderSS, () -> -LoaderConstants.POWER)));
        operatorXbox.getXBtn().whileHeld(new MoveMovableSubsystem(loaderSS, () -> -LoaderConstants.POWER));
    }

    /**
     * Puts the subsystems and commands in the dashboard
     */
    private void putData() {
        SmartDashboard.putData("Swerve", swerveSS);
        SmartDashboard.putData("Pitcher", pitcherSS);
        SmartDashboard.putData("Climber", climberSS);
        SmartDashboard.putData("Shooter", shooterSS);
        SmartDashboard.putData("ShootCG", shootCG);
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
            new ClimbCMD(climberSS, ClimberConstants.MAX_LEFT_POSITION, ClimberConstants.MAX_RIGHT_POSITION).schedule();
        else if(driverXbox.getLeftTriggerAxis() > ClimberConstants.TRIGGER_DEADBAND)
            new ClimbCMD(
                    climberSS, -ClimberConstants.MAX_LEFT_POSITION, -ClimberConstants.MAX_RIGHT_POSITION).schedule();
        else if(operatorXbox.getPOV() == 270)
            new ClimbCMD(climberSS, 0, 0).schedule();

        if(operatorXbox.getYButton()) {
            if(operatorXbox.getRightBumper())
                climberSS.moveRight(-ClimberConstants.OVERRIDDEN_POWER);
            if(operatorXbox.getLeftBumper())
                climberSS.moveLeft(-ClimberConstants.OVERRIDDEN_POWER);
        } else {
            if(operatorXbox.getRightBumper())
                climberSS.moveRight(ClimberConstants.OVERRIDDEN_POWER);
            if(operatorXbox.getLeftBumper())
                climberSS.moveLeft(ClimberConstants.OVERRIDDEN_POWER);
        }
        if(!operatorXbox.getRightBumper())
            climberSS.moveRight(0);
        if(!operatorXbox.getLeftBumper())
            climberSS.moveLeft(0);
    }

    public boolean isEndgame() {
        return endgame;
    }

    public void setEndgame(boolean endgame) {
        this.endgame = endgame;
    }
}
