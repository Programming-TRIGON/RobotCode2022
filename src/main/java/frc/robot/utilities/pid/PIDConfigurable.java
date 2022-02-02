package frc.robot.utilities.pid;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public interface PIDConfigurable extends Sendable {

    PIDCoefs getCoefs();

    double getSetpoint();

    void setSetpoint(double setpoint);

    boolean isTuning();

    void setIsTuning(boolean isTuning);

    default double getKP() {
        return getCoefs().getKP();
    }

    void setKP(double KP);

    default double getKI() {
        return getCoefs().getKI();
    }

    void setKI(double KI);

    default double getKD() {
        return getCoefs().getKD();
    }

    void setKD(double KD);

    default double getTolerance() {
        return getCoefs().getTolerance();
    }

    void setTolerance(double tolerance);

    default double getDeltaTolerance() {
        return getCoefs().getDeltaTolerance();
    }

    void setDeltaTolerance(double deltaTolerance);

    @Override
    default void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("RobotPreferences");
        // sends the pid values to the dashboard but only allows them to be changed if
        // isTuning is true
        builder.addDoubleProperty("p", this::getKP, kP -> setKP(isTuning() ? kP : getKP()));
        builder.addDoubleProperty("i", this::getKI, kI -> setKI(isTuning() ? kI : getKI()));
        builder.addDoubleProperty("d", this::getKD, kD -> setKD(isTuning() ? kD : getKD()));
        builder.addDoubleProperty(
                "tolerance", getCoefs()::getTolerance,
                tolerance -> setTolerance(isTuning() ? tolerance : getTolerance()));
        builder.addDoubleProperty(
                "deltaTolerance", getCoefs()::getDeltaTolerance,
                deltaTolerance -> setDeltaTolerance(isTuning() ? deltaTolerance : getDeltaTolerance()));
        builder.addDoubleProperty("setpoint", this::getSetpoint,
                setpoint -> setSetpoint(isTuning() ? setpoint : getSetpoint()));
        builder.addBooleanProperty("isTuning", this::isTuning, this::setIsTuning);
    }
}
