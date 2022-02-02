package frc.robot.utilities.pid;

import com.google.gson.annotations.SerializedName;

/**
 * This class is used to store settings for different PIDs
 */
public class PIDCoefs {
    @SerializedName(value = "kP", alternate = {"p", "P", "KP"})
    private double KP;
    @SerializedName(value = "kI", alternate = {"i", "I", "KI"})
    private double KI;
    @SerializedName(value = "kD", alternate = {"d", "D", "KD"})
    private double KD;
    @SerializedName(value = "tolerance", alternate = {"kT", "t", "T", "KT"})
    private double tolerance;
    @SerializedName(value = "deltaTolerance", alternate = {"kDT", "dt", "DT", "KDT"})
    private double deltaTolerance;

    /**
     * @param KP             The Proportional coefficient of the PID loop in this
     *                       command.
     * @param KI             The Integral coefficient of the PID loop in this
     *                       command.
     * @param KD             The Differential coefficient of the PID loop in this
     *                       command.
     * @param tolerance      The error tolerance of this command.
     * @param deltaTolerance The tolerance of the change in error.
     */
    public PIDCoefs(double KP, double KI, double KD, double tolerance, double deltaTolerance) {
        this.KP = KP;
        this.KI = KI;
        this.KD = KD;
        this.tolerance = tolerance;
        this.deltaTolerance = deltaTolerance;
    }

    /**
     * Default constructor
     */
    public PIDCoefs() {
    }

    public double getKP() {
        return KP;
    }

    public void setKP(double KP) {
        this.KP = KP;
    }

    public double getKI() {
        return KI;
    }

    public void setKI(double KI) {
        this.KI = KI;
    }

    public double getKD() {
        return KD;
    }

    public void setKD(double KD) {
        this.KD = KD;
    }

    public double getTolerance() {
        return tolerance;
    }

    public void setTolerance(double tolerance) {
        this.tolerance = tolerance;
    }

    public double getDeltaTolerance() {
        return deltaTolerance;
    }

    public void setDeltaTolerance(double deltaTolerance) {
        this.deltaTolerance = deltaTolerance;
    }
}