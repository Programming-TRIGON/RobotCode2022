package frc.robot.subsystems.climber;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.components.TrigonTalonSRX;
import frc.robot.constants.RobotConstants.ClimberConstants;
import frc.robot.subsystems.OverridableSubsystem;
import frc.robot.utilities.EncoderConversions;

import javax.swing.text.Position;

public class ClimberSS extends OverridableSubsystem {
    private TrigonTalonSRX rightMotor = ClimberConstants.RIGHT_MOTOR;
    private TrigonTalonSRX leftMotor = ClimberConstants.LEFT_MOTOR;

    // TODO: 17/02/2022 make a method that does setPosition, And a method that does override to Overridenmove

    public ClimberSS() {

    }



    public void setPosition(double desiredDestination) {
        double destinationClamp = MathUtil.clamp(desiredDestination, ClimberConstants.lowBoundary, ClimberConstants.highBoundary);
        //rightMotor.set(ControlMode.Position, EncoderConversions);
    }

    @Override
    public void overriddenMove(double power) {
        rightMotor.set(power);
        leftMotor.set(power);
    }
}
