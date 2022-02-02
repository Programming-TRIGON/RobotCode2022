package frc.robot.utilities;

public class OIMap {
    public enum Button {
        rightStick(1), leftStick(2), right1(3), right2(4), right3(5), left1(6), left2(7), left3(9), xButton(10),
        squareButton(11), triangleButton(12), circleButton(13), dPadUpButton(14), dPadRightButton(15),
        dPadDownButton(16), dPadLeftButton(17), touchPad(18), selectButton(19), startButton(20), playStationButton(21);

        private final int port;

        Button(int port) {
            this.port = port;
        }

        public int getPort() {
            return port;
        }
    }

    public enum Axis {
        leftX(0), rightX(1), leftY(2), rightY(3), leftTrigger(4), rightTrigger(5);

        private final int axis;

        Axis(int axis) {
            this.axis = axis;
        }

        public int getAxis() {
            return axis;
        }
    }
}