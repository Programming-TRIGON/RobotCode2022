package frc.robot.subsystems.shooter;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CharacterizationConstants;
import frc.robot.constants.RobotConstants.ShooterConstants;
import frc.robot.subsystems.CharacterizableSubsystem;
import frc.robot.subsystems.PIDFSubsystem;
import frc.robot.utilities.Conversions;
import frc.robot.utilities.pid.PIDFTalonSRX;

public class ShooterSS extends SubsystemBase implements PIDFSubsystem, CharacterizableSubsystem {
    private final PIDFTalonSRX masterMotor;

    public double getDistanceToCalculate() {
        return distanceToCalculate;
    }

    public void setDistanceToCalculate(double distanceToCalculate) {
        this.distanceToCalculate = distanceToCalculate;
    }

    private double distanceToCalculate;

    public ShooterSS() {
        masterMotor = ShooterConstants.LEFT_MOTOR;

        ShooterConstants.LEFT_MOTOR.follow(masterMotor);
        ShooterConstants.RIGHT_MOTOR.follow(masterMotor);

        putCharacterizeCMDInDashboard();
        distanceToCalculate = 0;
    }

    /**
     * @param setpoint the desired velocity in RPM
     */
    @Override
    public void setSetpoint(double setpoint) {
        masterMotor.setSetpoint(Conversions.RPMToFalcon(setpoint));
    }

    public double getSetpoint() {
        return Conversions.falconToRPM(masterMotor.getSetpoint());
    }

    @Override
    public boolean atSetpoint() {
        return masterMotor.atSetpoint();
    }

    /**
     * @param power to be set to the motors
     */
    @Override
    public void move(double power) {
        masterMotor.set(power);
    }

    /**
     * @return the velocity of the motors in RPM
     */
    public double getVelocity() {
        return Conversions.falconToRPM(masterMotor.getSelectedSensorVelocity());
    }

    /**
     * sets the feedforward for all the different components of the subsystem
     *
     * @param kV velocity gains
     * @param kS static gains
     */
    @Override
    public void updateFeedforward(double[] kV, double[] kS) {
        masterMotor.setKV(Conversions.kVToTalon(kV[0]));
        masterMotor.setKS(kS[0]);
    }

    @Override
    public CharacterizationConstants getCharacterizationConstants() {
        return ShooterConstants.CHARACTERIZATION_CONSTANTS;
    }

    /**
     * @return an array of the current encoder position
     */
    @Override
    public double[] getValues() {
        return new double[] {masterMotor.getSelectedSensorVelocity()};
    }

    @Override
    public String getName() {
        return "Shooter";
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("List");

        builder.addDoubleProperty("RPM", this::getVelocity, null);

        builder.addDoubleProperty("setpoint", this::getSetpoint, this::setSetpoint);
        builder.addDoubleProperty("distance", this::getDistanceToCalculate, this::setDistanceToCalculate);
        builder.addDoubleProperty(
                "velocity to shoot", () -> ShootingCalculations.calculateVelocity(distanceToCalculate), null);
        builder.addDoubleProperty("angle", () -> ShootingCalculations.calculateAngle(distanceToCalculate), null);

        SmartDashboard.putData("Shooter/Master Motor", masterMotor);
    }
}
