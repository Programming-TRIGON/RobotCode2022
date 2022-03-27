package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.MoveMovableSubsystem;
import frc.robot.commands.PIDCommand;
import frc.robot.commands.RunWhenDisabledCommand;
import frc.robot.commands.commandgroups.ClimbCG;
import frc.robot.commands.commandgroups.ShootCG;
import frc.robot.commands.commandgroups.ShootFromCloseCG;
import frc.robot.components.TrigonXboxController;
import frc.robot.constants.RobotConstants.*;
import frc.robot.subsystems.climber.ClimbCMD;
import frc.robot.subsystems.climber.ClimberSS;
import frc.robot.subsystems.intake.IntakeOpenerSS;
import frc.robot.subsystems.intake.IntakeSS;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.loader.LoaderSS;
import frc.robot.subsystems.shooter.*;
import frc.robot.subsystems.swerve.SupplierDriveCMD;
import frc.robot.subsystems.swerve.SwerveSS;
import frc.robot.subsystems.transporter.TransporterSS;
import frc.robot.utilities.DashboardController;
import frc.robot.vision.CamMode;
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
    private ShootCG shootCG;

    // States
    private boolean endgame = false;

    private final Button userBtn;
    private final Button rightTrigger;
    private final Button leftTrigger;

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

        cargoLimelight = new Limelight("limelight-cargo");
        setCargoLimelightPipeline();

        hubLimelight = new Limelight("limelight-hub");
        hubLimelight.setCamMode(CamMode.vision);

        userBtn = new Button(RobotController::getUserButton);
        rightTrigger = new Button(() -> driverXbox.getRightTriggerAxis() > ClimberConstants.TRIGGER_DEADBAND);
        leftTrigger = new Button(() -> driverXbox.getLeftTriggerAxis() > ClimberConstants.TRIGGER_DEADBAND);

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
                () -> (driverXbox.getLeftX() + commanderXbox.getLeftX()) /
                        (isEndgame() ?
                         DriverConstants.ENDGAME_SPEED_DIVIDER :
                         (intakeSS.isCollecting() ?
                          DriverConstants.COLLECT_SPEED_DIVIDER :
                          DriverConstants.SPEED_DIVIDER)),
                () -> (driverXbox.getLeftY() + commanderXbox.getLeftY()) /
                        (isEndgame() ?
                         DriverConstants.ENDGAME_SPEED_DIVIDER :
                         (intakeSS.isCollecting() ?
                          DriverConstants.COLLECT_SPEED_DIVIDER :
                          DriverConstants.SPEED_DIVIDER)),
                () -> (driverXbox.getRightX() + commanderXbox.getRightX()) /
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
                new WaitCommand(1.5).andThen(
                        new ParallelCommandGroup(
                                new MoveMovableSubsystem(intakeSS, () -> IntakeConstants.POWER),
                                new MoveMovableSubsystem(transporterSS, () -> TransporterConstants.POWER))));
        driverXbox.getLeftBumperBtn().whenReleased(
                new ParallelCommandGroup(
                        new InstantCommand(() -> intakeOpenerSS.setState(false)),
                        new WaitCommand(0.3)
                                .andThen(new MoveMovableSubsystem(loaderSS, () -> -LoaderConstants.POWER).withTimeout(
                                        0.2)))
        );
        driverXbox.getBBtn().whileHeld(shootCG);
        driverXbox.getRightBumperBtn().whileHeld(new MoveMovableSubsystem(loaderSS, () -> LoaderConstants.POWER));

        // Shoot from close
        SendableChooser<CloseVelocitiesAndAngles> closeWaypoint = new SendableChooser<>();
        closeWaypoint.setDefaultOption(CloseVelocitiesAndAngles.first.name(), CloseVelocitiesAndAngles.first);
        for(CloseVelocitiesAndAngles waypoint : CloseVelocitiesAndAngles.values()) {
            closeWaypoint.addOption(waypoint.name(), waypoint);
        }
        SmartDashboard.putData("closeeee", closeWaypoint);
        driverXbox.getABtn()
                .whileHeld(new ShootFromCloseCG(this, () -> closeWaypoint.getSelected().getVelocity(),
                        () -> closeWaypoint.getSelected().getAngle()));
        //        driverXbox.getXBtn().whenPressed(new InstantCommand(() -> {
        //            if(shooterSS.getOutput() == 0) {
        //                shooterSS.setSetpoint(closeWaypoint.getSelected().getVelocity());
        //                pitcherSS.setSetpoint(closeWaypoint.getSelected().getAngle());
        //            } else {
        //                shooterSS.stopMoving();
        //            }
        //        }));

        //        driverXbox.getXBtn().whileHeld(new IntakeCG(this))
        //                .whenReleased(new MoveMovableSubsystem(intakeSS, () -> IntakeConstants.POWER).withTimeout(0.4)
        //                        .andThen(new MoveMovableSubsystem(loaderSS, () -> -LoaderConstants.POWER).withTimeout(
        //                                0.2)).andThen().andThen(new InstantCommand(() -> intakeOpenerSS.setState
        //                                (false))));
        rightTrigger.whileHeld(
                new ClimbCMD(
                        climberSS, ClimberConstants.MAX_LEFT_POSITION, ClimberConstants.MAX_RIGHT_POSITION,
                        this::isEndgame));
        leftTrigger.whileHeld(
                new ClimbCMD(climberSS, 0, 0, this::isEndgame));
        commanderXbox.getBBtn().whileHeld(new SequentialCommandGroup(
                new InstantCommand(() -> intakeOpenerSS.setState(true)),
                new WaitCommand(1.5),
                new ParallelCommandGroup(
                        new MoveMovableSubsystem(intakeSS, () -> -IntakeConstants.POWER),
                        new MoveMovableSubsystem(transporterSS, () -> -TransporterConstants.POWER),
                        new MoveMovableSubsystem(loaderSS, () -> -LoaderConstants.POWER))));
        commanderXbox.getBBtn().whenReleased(new InstantCommand(() -> intakeOpenerSS.setState(false)));
        commanderXbox.getXBtn().whileHeld(new MoveMovableSubsystem(loaderSS, () -> -LoaderConstants.POWER));
        commanderXbox.getABtn().whenPressed(new InstantCommand(() -> setEndgame(!isEndgame())));
        commanderXbox.getYBtn().whileHeld(
                (new ParallelCommandGroup(
                        new ShootCMD(
                                shooterSS,
                                () -> closeWaypoint.getSelected().getVelocity()),
                        new PIDCommand(
                                pitcherSS,
                                () -> closeWaypoint.getSelected().getAngle())
                )).withInterrupt(this::isEndgame));
        commanderXbox.getStartBtn().toggleWhenPressed(new ClimbCG(this, this::isEndgame));

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
        //        if(driverXbox.getRightTriggerAxis() > ClimberConstants.TRIGGER_DEADBAND)
        //            new ClimbCMD(climberSS, ClimberConstants.MAX_LEFT_POSITION, ClimberConstants
        //            .MAX_RIGHT_POSITION).schedule();
        //        else if(driverXbox.getLeftTriggerAxis() > ClimberConstants.TRIGGER_DEADBAND)
        //            new ClimbCMD(
        //                    climberSS, -ClimberConstants.MAX_LEFT_POSITION, -ClimberConstants.MAX_RIGHT_POSITION)
        //                    .schedule();
        //        else if(commanderXbox.getPOV() == 270)
        //            new ClimbCMD(climberSS, 0, 0).schedule();
        if(!rightTrigger.getAsBoolean() && !leftTrigger.getAsBoolean() && climberSS.getCurrentCommand() == null) {
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
            if(!commanderXbox.getRightBumper() && climberSS.getCurrentCommand() == null)
                climberSS.moveRight(0);
            if(!commanderXbox.getLeftBumper() && climberSS.getCurrentCommand() == null)
                climberSS.moveLeft(0);
        }
    }

    public boolean isEndgame() {
        return endgame;
    }

    public void setEndgame(boolean endgame) {
        this.endgame = endgame;
    }

    public void setCargoLimelightPipeline() {
        cargoLimelight.setPipeline(DriverStation.getAlliance().equals(DriverStation.Alliance.Red) ? 0 : 1);
        cargoLimelight.setCamMode(CamMode.vision);
    }
}
