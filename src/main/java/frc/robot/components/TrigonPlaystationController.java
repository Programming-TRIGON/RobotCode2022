package frc.robot.components;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.constants.RobotConstants.DriverConstants;
import frc.robot.utilities.OIMap;
import frc.robot.utilities.OIMap.Axis;

public class TrigonPlaystationController extends GenericHID {
    private final JoystickButton right1;
    private final JoystickButton right2;
    private final JoystickButton right3;
    private final JoystickButton left1;
    private final JoystickButton left2;
    private final JoystickButton left3;
    private final JoystickButton xButton;
    private final JoystickButton squareButton;
    private final JoystickButton triangleButton;
    private final JoystickButton circleButton;
    private final JoystickButton dPadUpButton;
    private final JoystickButton dPadRightButton;
    private final JoystickButton dPadDownButton;
    private final JoystickButton dPadLeftButton;
    private final JoystickButton touchPad;
    private final JoystickButton selectButton;
    private final JoystickButton startButton;
    private final JoystickButton playStationButton;
    private final Notifier notifier;
    private int rumbleAmount;

    public TrigonPlaystationController(int port) {
        super(port);
        right1 = new JoystickButton(this, OIMap.Button.right1.getPort());
        right2 = new JoystickButton(this, OIMap.Button.right2.getPort());
        right3 = new JoystickButton(this, OIMap.Button.right3.getPort());
        left1 = new JoystickButton(this, OIMap.Button.left1.getPort());
        left2 = new JoystickButton(this, OIMap.Button.left2.getPort());
        left3 = new JoystickButton(this, OIMap.Button.left3.getPort());
        xButton = new JoystickButton(this, OIMap.Button.xButton.getPort());
        squareButton = new JoystickButton(this, OIMap.Button.squareButton.getPort());
        triangleButton = new JoystickButton(this, OIMap.Button.triangleButton.getPort());
        circleButton = new JoystickButton(this, OIMap.Button.circleButton.getPort());
        dPadUpButton = new JoystickButton(this, OIMap.Button.dPadUpButton.getPort());
        dPadRightButton = new JoystickButton(this, OIMap.Button.dPadRightButton.getPort());
        dPadDownButton = new JoystickButton(this, OIMap.Button.dPadDownButton.getPort());
        dPadLeftButton = new JoystickButton(this, OIMap.Button.dPadLeftButton.getPort());
        touchPad = new JoystickButton(this, OIMap.Button.touchPad.getPort());
        selectButton = new JoystickButton(this, OIMap.Button.selectButton.getPort());
        startButton = new JoystickButton(this, OIMap.Button.startButton.getPort());
        playStationButton = new JoystickButton(this, OIMap.Button.playStationButton.getPort());
        rumbleAmount = -1;
        notifier = new Notifier(this::notifierPeriodic);
    }

    public JoystickButton getRight1() {
        return right1;
    }

    public JoystickButton getRight2() {
        return right2;
    }

    public JoystickButton getRight3() {
        return right3;
    }

    public JoystickButton getLeft1() {
        return left1;
    }

    public JoystickButton getLeft2() {
        return left2;
    }

    public JoystickButton getLeft3() {
        return left3;
    }

    public JoystickButton getXButton() {
        return xButton;
    }

    public JoystickButton getSquareButton() {
        return squareButton;
    }

    public JoystickButton getTriangleButton() {
        return triangleButton;
    }

    public JoystickButton getCircleButton() {
        return circleButton;
    }

    public JoystickButton getDpadUpButton() {
        return dPadUpButton;
    }

    public JoystickButton getDpadRightButton() {
        return dPadRightButton;
    }

    public JoystickButton getDpadDownButton() {
        return dPadDownButton;
    }

    public JoystickButton getDpadLeftButton() {
        return dPadLeftButton;
    }

    public JoystickButton getTouchPad() {
        return touchPad;
    }

    public JoystickButton getSelectButton() {
        return selectButton;
    }

    public JoystickButton getStartButton() {
        return startButton;
    }

    public JoystickButton getPlayStatButton() {
        return playStationButton;
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

    public double getLeftX() {
        return getRawAxis(Axis.leftX.getAxis());
    }

    public double getLeftY() {
        return getRawAxis(Axis.leftY.getAxis());
    }

    public double getRightX() {
        return getRawAxis(Axis.rightX.getAxis());
    }

    public double getRightY() {
        return getRawAxis(Axis.rightY.getAxis());
    }

    public double deltaTriggers() {
        return getRawAxis(Axis.rightTrigger.getAxis()) - getRawAxis(Axis.leftTrigger.getAxis());
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
}
