package frc.robot.swerve;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.util.Conversions;
import frc.robot.util.SwerveModuleConstants;

public class TalonFXSwerveModule extends SwerveModule {
    private TalonFX angleMotor;
    private TalonFX driveMotor;
    
    private PositionVoltage angleControl;
    private VelocityVoltage driveControl;

    public TalonFXSwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
        super(moduleNumber, moduleConstants);

        /* Drive Motor Config */
        driveMotor = new TalonFX(moduleConstants.driveMotorID);
        configDriveMotor();

        /* Angle Motor Config */
        angleMotor = new TalonFX(moduleConstants.angleMotorID);
        configAngleMotor();
        
        angleControl = new PositionVoltage(0);
        angleControl.Slot = 0;
        driveControl = new VelocityVoltage(0);
        driveControl.Slot = 0;
    }

    @Override
    public void resetToAbsolute() {
        super.resetToAbsolute();
        double absolutePosition = Conversions.degreesToRotations(getCanCoder().getDegrees() - angleOffset.getDegrees());
        angleMotor.setPosition(absolutePosition);
    }

    private void configAngleMotor() {
        angleMotor.getConfigurator().apply(Constants.Swerve.CTRE_CONFIGS.swerveAngleConfig);
        angleMotor.setInverted(Constants.Swerve.INVERT_ANGLE_MOTORS);
        angleMotor.setNeutralMode(Constants.Swerve.ANGLE_NEUTRAL_MODE);
        resetToAbsolute();
    }

    private void configDriveMotor() {       
        driveMotor.getConfigurator().apply(Constants.Swerve.CTRE_CONFIGS.swerveDriveConfig); 
        driveMotor.setInverted(Constants.Swerve.INVERT_DRIVE_MOTORS);
        driveMotor.setNeutralMode(Constants.Swerve.DRIVE_NEUTRAL_MODE);
        driveMotor.setPosition(0);
    }

    @Override
    public void setAngle(SwerveModuleState desiredState) {
        // Prevent rotating module if speed is less then 1%. Prevents Jittering.
        // Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.MAX_SPEED * 0.01)) ? lastAngle : desiredState.angle; 
        var angle = desiredState.angle;
        angleControl.withPosition(Conversions.degreesToRotations(angle.getDegrees()));
        SmartDashboard.putNumber("testtestsetes", angleControl.Position);
        SmartDashboard.putNumber("testtestsetes2", angle.getDegrees());
        SmartDashboard.putNumber("testtestsetes3", getAngle().getDegrees());
        angleMotor.setControl(angleControl);
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
            
            driveControl.withVelocity(velocity)
                    .withFeedForward(feedforward.calculate(desiredState.speedMetersPerSecond));
            driveMotor.setControl(driveControl);
        }
    }

    @Override
    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(Conversions.rotationsToDegrees(angleMotor.getPosition().getValue()));
    }

    @Override
    public Rotation2d getCanCoder() {
        return Rotation2d.fromDegrees(Conversions.rotationsToDegrees(angleEncoder.getAbsolutePosition().getValue()));
    }

    @Override
    public SwerveModuleState getState() {
        return new SwerveModuleState(
            Conversions.RPSToMPS(driveMotor.getVelocity().getValue(), Constants.Swerve.WHEEL_CIRCUMFERENCE), 
            getAngle()
        ); 
    }

    @Override
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            Conversions.rotationsToMeters(driveMotor.getPosition().getValue(), Constants.Swerve.WHEEL_CIRCUMFERENCE), 
            getAngle()
        );
    }
}
