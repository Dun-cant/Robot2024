package frc.robot.swerve;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.util.Conversions;
import frc.robot.util.SwerveModuleConstants;

public class SparkMaxSwerveModule extends SwerveModule {
    private CANSparkMax angleMotor;
    private CANSparkMax driveMotor;

    private RelativeEncoder angleRelativeEncoder;
    private RelativeEncoder driveRelativeEncoder;

    private SparkPIDController angleController;
    private SparkPIDController driveController;

    public SparkMaxSwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
        super(moduleNumber, moduleConstants);

        /* Angle Motor Config */
        angleMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
        configAngleMotor();
        angleRelativeEncoder = angleMotor.getEncoder();
        angleController = angleMotor.getPIDController();

        /* Drive Motor Config */
        driveMotor = new CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
        configDriveMotor();
        driveRelativeEncoder = driveMotor.getEncoder();
        driveController = driveMotor.getPIDController();
    }

    @Override
    public void resetToAbsolute() {
        double absolutePosition = Conversions.degreesToRotations(getCanCoder().getDegrees() - angleOffset.getDegrees());
        angleEncoder.setPosition(absolutePosition);
    }

    private void configAngleMotor() {
        angleMotor.restoreFactoryDefaults();

        angleMotor.setSmartCurrentLimit(Constants.Swerve.ANGLE_CONTINUOUS_CURRENT_LIMIT);
        angleController.setP(Constants.Swerve.ANGLE_P);
        angleController.setI(Constants.Swerve.ANGLE_I);
        angleController.setD(Constants.Swerve.ANGLE_D);
        angleController.setFF(Constants.Swerve.ANGLE_F);

        angleMotor.setIdleMode(Constants.Swerve.ANGLE_IDLE_MODE);
        angleRelativeEncoder.setPositionConversionFactor(1/Constants.Swerve.ANGLE_GEAR_RATIO);
        angleRelativeEncoder.setVelocityConversionFactor(1/Constants.Swerve.ANGLE_GEAR_RATIO);

        angleMotor.setInverted(Constants.Swerve.INVERT_ANGLE_MOTORS);
        resetToAbsolute();
        
    }

    private void configDriveMotor() {
        driveMotor.restoreFactoryDefaults();  
        driveMotor.setInverted(Constants.Swerve.INVERT_DRIVE_MOTORS);

        driveMotor.setSmartCurrentLimit(Constants.Swerve.DRIVE_CONTINUOUS_CURRENT_LIMIT);
        driveController.setP(Constants.Swerve.DRIVE_P);
        driveController.setI(Constants.Swerve.DRIVE_I);
        driveController.setD(Constants.Swerve.DRIVE_D);
        driveController.setFF(Constants.Swerve.DRIVE_F);
        
        driveMotor.setIdleMode(Constants.Swerve.ANGLE_IDLE_MODE);
        driveRelativeEncoder.setPositionConversionFactor(1/Constants.Swerve.DRIVE_GEAR_RATIO);
        driveRelativeEncoder.setVelocityConversionFactor(1/Constants.Swerve.DRIVE_GEAR_RATIO);
        driveMotor.setOpenLoopRampRate(Constants.Swerve.OPEN_LOOP_RAMP);
        driveMotor.setClosedLoopRampRate(Constants.Swerve.CLOSED_LOOP_RAMP);
        
        driveRelativeEncoder.setPosition(0);
    }

    @Override
    public void setAngle(SwerveModuleState desiredState) {
        // Prevent rotating module if speed is less then 1%. Prevents Jittering.
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.MAX_SPEED * 0.01)) ? lastAngle : desiredState.angle; 
        
        angleController.setReference(Conversions.degreesToRotations(angle.getDegrees()), ControlType.kPosition);
        lastAngle = angle;
    }

    @Override
    public void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.MAX_SPEED;
           
            driveMotor.set(percentOutput);
        }
        else {
            double velocity = Conversions.MPSToRPS(desiredState.speedMetersPerSecond, Constants.Swerve.WHEEL_CIRCUMFERENCE);
            
            driveController.setReference(Conversions.RPSToRPM(velocity), ControlType.kVelocity, 0, feedforward.calculate(desiredState.speedMetersPerSecond));

        }
    }

    @Override
    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(Conversions.rotationsToDegrees(angleRelativeEncoder.getPosition()));
    }

    @Override
    public Rotation2d getCanCoder() {
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition().getValue());
    }

    @Override
    public SwerveModuleState getState() {
        return new SwerveModuleState(
            Conversions.RPSToMPS(driveRelativeEncoder.getVelocity(), Constants.Swerve.WHEEL_CIRCUMFERENCE), 
            getAngle()
        ); 
    }

    @Override
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            Conversions.rotationsToMeters(driveRelativeEncoder.getPosition(), Constants.Swerve.WHEEL_CIRCUMFERENCE), 
            getAngle()
        );
    }
}
