package frc.robot.subsystems.shooter;

public enum CloseVelocitiesAndAngles {
    first(2500, 1);

    private final double velocity;
    private final double angle;

    CloseVelocitiesAndAngles(double velocity, double angle) {
        this.velocity = velocity;
        this.angle = angle;
    }

    public double getVelocity() {
        return velocity;
    }

    public double getAngle() {
        return angle;
    }
}
