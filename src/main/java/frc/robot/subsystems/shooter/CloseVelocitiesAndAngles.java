package frc.robot.subsystems.shooter;

public enum CloseVelocitiesAndAngles {
    first(2500, 1),
    second(2400, 3),
    third(2450, 1),
    fourth(2500, 1.5),
    five(2500, 2.5),
    bothActLikeFirstBall(2500, 0);

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
