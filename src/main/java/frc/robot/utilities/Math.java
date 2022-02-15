package frc.robot.utilities;

public class Math {
    public static double signedSquare(double x) {
        return x * x * java.lang.Math.signum(x);
    }
}
