package frc.robot.constants;

import frc.robot.components.TrigonTalonSRX;
import frc.robot.utilities.pid.PIDFTalonFX;

public class SwerveModuleConstants {
    public final PIDFTalonFX angleMotor;
    public final PIDFTalonFX driveMotor;
    public final TrigonTalonSRX angleEncoder;
    public final double encoderOffset;

    public SwerveModuleConstants(
            PIDFTalonFX angleMotor, PIDFTalonFX driveMotor, TrigonTalonSRX angleEncoder, double encoderOffset) {
        this.angleMotor = angleMotor;
        this.driveMotor = driveMotor;
        this.angleEncoder = angleEncoder;
        this.encoderOffset = encoderOffset;
    }
}

