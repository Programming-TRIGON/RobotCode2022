package frc.robot.components;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.constants.RobotConstants.DriverConstants;

public class TrigonXboxController extends XboxController {
    private final JoystickButton aBtn;
    private final JoystickButton bBtn;
    private final JoystickButton xBtn;
    private final JoystickButton yBtn;
    private final JoystickButton leftBumperBtn;
    private final JoystickButton rightBumperBtn;
    private final JoystickButton leftStickButtonBtn;
    private final JoystickButton rightStickButtonBtn;
    private final JoystickButton backBtn;
    private final JoystickButton startBtn;
    private final Notifier notifier;
    private int rumbleAmount;
    private final boolean squared;
    private final double deadband;

    public TrigonXboxController(int port, double deadband, boolean squared) {
        super(port);
        this.deadband = deadband;
        this.squared = squared;
        aBtn = new JoystickButton(this, Button.kA.value);
        bBtn = new JoystickButton(this, Button.kB.value);
        xBtn = new JoystickButton(this, Button.kX.value);
        yBtn = new JoystickButton(this, Button.kY.value);
        leftBumperBtn = new JoystickButton(this, Button.kLeftBumper.value);
        rightBumperBtn = new JoystickButton(this, Button.kRightBumper.value);
        leftStickButtonBtn = new JoystickButton(this, Button.kLeftStick.value);
        rightStickButtonBtn = new JoystickButton(this, Button.kRightStick.value);

        backBtn = new JoystickButton(this, Button.kBack.value);
        startBtn = new JoystickButton(this, Button.kStart.value);
        rumbleAmount = -1;
        notifier = new Notifier(this::notifierPeriodic);
    }

    public JoystickButton getABtn() {
        return aBtn;
    }

    public JoystickButton getBBtn() {
        return bBtn;
    }

    public JoystickButton getXBtn() {
        return xBtn;
    }

    public JoystickButton getYBtn() {
        return yBtn;
    }

    public JoystickButton getLeftBumperBtn() {
        return leftBumperBtn;
    }

    public JoystickButton getRightBumperBtn() {
        return rightBumperBtn;
    }

    public JoystickButton getLeftStickButtonBtn() {
        return leftStickButtonBtn;
    }

    public JoystickButton getRightStickButtonBtn() {
        return rightStickButtonBtn;
    }

    public JoystickButton getBackBtn() {
        return backBtn;
    }

    public JoystickButton getStartBtn() {
        return startBtn;
    }

    /**
     * Right trigger value - left trigger value.
     *
     * @return The difference between the left and right triggers.
     */
    public double getDeltaTriggers() {
        return getRightTriggerAxis() - getLeftTriggerAxis();
    }

    /**
     * Set the rumble output for the HID. this method affects both motors.
     *
     * @param power The normalized value (0 to 1) to set the rumble to
     */
    public void setRumble(double power) {
        setRumble(RumbleType.kLeftRumble, power);
        setRumble(RumbleType.kRightRumble, power);
    }

    /**
     * Rumbles the controller the desired amount of times.
     *
     * @param quantity the number of times to rumble.
     */
    public void intermittentRumble(int quantity) {
        rumbleAmount = quantity * 2 - 1;
        notifier.startPeriodic(DriverConstants.RUMBLE_INTERMISSION_TIME);
    }

    @Override
    public double getLeftX() {
        return processValue(super.getLeftX());
    }

    @Override
    public double getLeftY() {
        return processValue(super.getLeftY());
    }

    @Override
    public double getRightX() {
        return processValue(super.getRightX());
    }

    @Override
    public double getRightY() {
        return processValue(super.getRightY());
    }

    public void notifierPeriodic() {
        if(rumbleAmount == 0) {
            notifier.stop();
        }
        if(rumbleAmount >= 0) {
            if(rumbleAmount % 2 == 1)
                setRumble(1);
            else
                setRumble(0);
            rumbleAmount--;
        }
    }

    private static double signedSquare(double value) {
        return Math.signum(value) * Math.pow(Math.abs(value), 2);
    }

    /**
     * processValue:
     * If the value is within the deadband, return 0.
     * If squared is true, the value is squared, while keeping the sign.
     *
     * @param value the value from a stick
     * @return the value after processing
     */
    private double processValue(double value) {
        if(value < deadband && value > -deadband)
            return 0;
        return squared ? signedSquare(value) : value;
    }
}
