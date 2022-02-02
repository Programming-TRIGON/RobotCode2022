package frc.robot.utilities.pid;

import com.google.gson.annotations.SerializedName;

public class PIDFCoefs extends PIDCoefs {
    @SerializedName(value = "kS", alternate = {"s", "S", "KS"})
    private double KS;
    @SerializedName(value = "kV", alternate = {"v", "V", "KV"})
    private double KV;

    /**
     * @param KP             The Proportional coefficient of the PID loop in this
     *                       command.
     * @param KI             The Integral coefficient of the PID loop in this
     *                       command.
     * @param KD             The Differential coefficient of the PID loop in this
     *                       command.
     * @param KV             The Velocity gain coefficient of the Feedforward controller in this
     *                       command.
     * @param KS             The Static gain coefficient of the Feedforward controller in this
     *                       command.
     * @param tolerance      The error tolerance of this command.
     * @param deltaTolerance The tolerance of the change in error.
     */
    public PIDFCoefs(double KP, double KI, double KD, double KV, double KS, double tolerance, double deltaTolerance) {
        super(KP, KI, KD, tolerance, deltaTolerance);
        this.KV = KV;
        this.KS = KS;
    }

    /**
     * Default constructor
     */
    public PIDFCoefs() {
    }

    public double getKV() {
        return KV;
    }

    public void setKV(double KV) {
        this.KV = KV;
    }

    public double getKS() {
        return KS;
    }

    public void setKS(double KS) {
        this.KS = KS;
    }
}
