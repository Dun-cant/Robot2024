package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.swerve.SwerveModule;
import frc.robot.swerve.TalonFXSwerveModule;

public class Swerve extends SubsystemBase {
    public SwerveDrivePoseEstimator poseEstimator;
    public SwerveModule[] modules;
    public AHRS gyro;

    private boolean braking;

    private SlewRateLimiter limiterx, limitery;

    public Swerve() {
        braking = false;
        gyro = new AHRS(Port.kMXP);

        this.limiterx = new SlewRateLimiter(6);
        this.limitery = new SlewRateLimiter(6);

        modules = new SwerveModule[] {
            new TalonFXSwerveModule(0, Constants.Swerve.Mod0.MODULE),
            new TalonFXSwerveModule(1, Constants.Swerve.Mod1.MODULE),
            new TalonFXSwerveModule(2, Constants.Swerve.Mod2.MODULE),
            new TalonFXSwerveModule(3, Constants.Swerve.Mod3.MODULE)
        };


        /* By pausing init for a second before setting module offsets, we avoid a bug with inverting motors.
         * See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
         */
        Timer.delay(1.0);
        resetModulesToAbsolute();

        poseEstimator = new SwerveDrivePoseEstimator(Constants.Swerve.swerveKinematics, getYaw(), getModulePositions(), new Pose2d());

        zeroGyro();

    }

    public void drive(Translation2d translation, double rotation, boolean isOpenLoop) {
        drive(translation, rotation, isOpenLoop, false);
    }

    public void drive(Translation2d translation, double rotation, boolean isOpenLoop, boolean isRateLimited) {
        if(braking) return;
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                                isRateLimited ? limiterx.calculate(translation.getX()) : translation.getX(), 
                                isRateLimited ? limitery.calculate(translation.getY()) : translation.getY(), 
                                rotation, 
                                getYaw()
                            ));
                    
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.MAX_SPEED);

        for(SwerveModule mod : modules){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }    

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.MAX_SPEED);
        for(SwerveModule mod : modules){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }
    
    public boolean isBraking() {
        return braking;
    }

    public void resetOdometry(Pose2d pose) {
        poseEstimator.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public void toggleBrake() {
        if(braking) {
            braking = false;
            setModuleStates(new SwerveModuleState[]{
                new SwerveModuleState(Constants.Swerve.MAX_SPEED * 0.011, Rotation2d.fromDegrees(0)),
                new SwerveModuleState(Constants.Swerve.MAX_SPEED * 0.011, Rotation2d.fromDegrees(0)),
                new SwerveModuleState(Constants.Swerve.MAX_SPEED * 0.011, Rotation2d.fromDegrees(0)),
                new SwerveModuleState(Constants.Swerve.MAX_SPEED * 0.011, Rotation2d.fromDegrees(0))
            });
            setModuleStates(new SwerveModuleState[]{
                new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(0))
            });
            SmartDashboard.putBoolean("Is Braking", false);
        } else {
            braking = true;
            setModuleStates(new SwerveModuleState[]{
                new SwerveModuleState(Constants.Swerve.MAX_SPEED * 0.011, Rotation2d.fromDegrees(-45)),
                new SwerveModuleState(Constants.Swerve.MAX_SPEED * 0.011, Rotation2d.fromDegrees(45)),
                new SwerveModuleState(Constants.Swerve.MAX_SPEED * 0.011, Rotation2d.fromDegrees(45)),
                new SwerveModuleState(Constants.Swerve.MAX_SPEED * 0.011, Rotation2d.fromDegrees(-45))
            });
            setModuleStates(new SwerveModuleState[]{
                new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(-45))
            });
            SmartDashboard.putBoolean("Is Braking", true);
        }
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : modules){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : modules){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public void zeroGyro(){
        gyro.zeroYaw();
        resetOdometry(new Pose2d(0, 0, new Rotation2d()));
    }

    public Rotation2d getYaw() {
        // should be ccw+
        return Rotation2d.fromDegrees(Math.abs(gyro.getYaw()-180)-180);
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : modules){
            mod.resetToAbsolute();
        }
    }

    @Override
    public void periodic(){
        poseEstimator.update(getYaw(), getModulePositions());  
        SmartDashboard.putNumber("Roll", gyro.getRoll());
        SmartDashboard.putNumber("Heading", getYaw().getDegrees());
        for(SwerveModule mod : modules){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }
    }
}