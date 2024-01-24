package frc.robot.util;


import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import frc.robot.Constants;
import frc.robot.Constants.Swerve;

public final class CTREConfigs {
    public TalonFXConfiguration swerveAngleConfig;
    public TalonFXConfiguration swerveDriveConfig;
    public CANcoderConfiguration swerveCanCoderConfig;

    public CTREConfigs(){
        swerveAngleConfig = new TalonFXConfiguration();
        swerveDriveConfig = new TalonFXConfiguration();
        swerveCanCoderConfig = new CANcoderConfiguration();

        /* Swerve Angle Motor Configurations */
        swerveAngleConfig.CurrentLimits
                .withSupplyCurrentLimitEnable(Constants.Swerve.ANGLE_ENABLE_CURRENT_LIMIT)
                .withSupplyCurrentLimit(Constants.Swerve.ANGLE_CONTINUOUS_CURRENT_LIMIT)
                .withSupplyCurrentThreshold(Constants.Swerve.ANGLE_PEAK_CURRENT_LIMIT)
                .withSupplyTimeThreshold(Constants.Swerve.ANGLE_PEAK_CURRENT_DURATION);

        swerveAngleConfig.Slot0
                .withKP(Constants.Swerve.ANGLE_P)
                .withKI(Constants.Swerve.ANGLE_I)
                .withKD(Constants.Swerve.ANGLE_D);

        swerveAngleConfig.MotorOutput.NeutralMode = Constants.Swerve.ANGLE_NEUTRAL_MODE;
        swerveAngleConfig.Feedback.SensorToMechanismRatio = Constants.Swerve.ANGLE_GEAR_RATIO;

        /* Swerve Drive Motor Configuration */
        swerveDriveConfig.CurrentLimits
                .withSupplyCurrentLimitEnable(Constants.Swerve.DRIVE_ENABLE_CURRENT_LIMIT)
                .withSupplyCurrentLimit(Constants.Swerve.DRIVE_CONTINUOUS_CURRENT_LIMIT)
                .withSupplyCurrentThreshold(Constants.Swerve.DRIVE_PEAK_CURRENT_LIMIT)
                .withSupplyTimeThreshold(Constants.Swerve.DRIVE_PEAK_CURRENT_DURATION);

        swerveDriveConfig.Slot0
                .withKP(Constants.Swerve.DRIVE_P)
                .withKI(Constants.Swerve.DRIVE_I)
                .withKD(Constants.Swerve.DRIVE_D);

        swerveDriveConfig.MotorOutput.NeutralMode = Constants.Swerve.DRIVE_NEUTRAL_MODE;
        swerveDriveConfig.Feedback.SensorToMechanismRatio = Constants.Swerve.DRIVE_GEAR_RATIO;
        
        swerveDriveConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = Constants.Swerve.OPEN_LOOP_RAMP;
        swerveDriveConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.Swerve.CLOSED_LOOP_RAMP;
        
        /* Swerve CANCoder Configuration */
        swerveCanCoderConfig.MagnetSensor
                .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1)
                .withSensorDirection(SensorDirectionValue.Clockwise_Positive);

        
    }
}