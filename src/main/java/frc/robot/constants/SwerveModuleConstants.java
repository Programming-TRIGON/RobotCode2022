package frc.robot.constants;

import frc.robot.components.TrigonTalonSRX;
import frc.robot.constants.LocalConstants.LocalSwerveConstants.LocalSwerveModules.LocalSwerveModuleConstants;
import frc.robot.utilities.Module;
import frc.robot.utilities.pid.PIDFTalonFX;

public class SwerveModuleConstants {
    public final PIDFTalonFX angleMotor;
    public final PIDFTalonFX driveMotor;
    public final TrigonTalonSRX angleEncoder;
    public final LocalSwerveModuleConstants localConstants;
    public final Module module;

    public SwerveModuleConstants(
            PIDFTalonFX angleMotor, PIDFTalonFX driveMotor, TrigonTalonSRX angleEncoder,
            LocalSwerveModuleConstants localConstants, Module module) {
        this.angleMotor = angleMotor;
        this.driveMotor = driveMotor;
        this.angleEncoder = angleEncoder;
        this.localConstants = localConstants;
        this.module = module;
    }
}

