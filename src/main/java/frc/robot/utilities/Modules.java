package frc.robot.utilities;

public enum Modules {
    FRONT_LEFT(0, "Front Left"),
    FRONT_RIGHT(1, "Front Right"),
    REAR_LEFT(2, "Rear Left"),
    REAR_RIGHT(3, "Rear Right");

    private final int id;
    private final String name;

    Modules(int id, String name) {
        this.id = id;
        this.name = name;
    }

    public int getId() {
        return id;
    }

    public String getName() {
        return name;
    }
}