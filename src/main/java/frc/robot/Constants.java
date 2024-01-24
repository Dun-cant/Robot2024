package frc.robot;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.util.COTSSwerveConstants;
import frc.robot.util.CTREConfigs;
import frc.robot.util.SwerveModuleConstants;

public final class Constants {
    public static final double STICK_DEADBAND = 0.1;

    public static final class Swerve {
        public static final COTSSwerveConstants CHOSEN_MODULE =  //TODO: This must be tuned to specific robot
            COTSSwerveConstants.SDSMK4i(COTSSwerveConstants.driveGearRatios.SDSMK4i_L1);

        /* Drivetrain Constants */
        /** Meters */
        public static final double TRACK_WIDTH = 0.50165; //TODO: This must be tuned to specific robot
        /** Meters */
        public static final double WHEEL_BASE = 0.50165; //TODO: This must be tuned to specific robot
        /** Meters */
        public static final double WHEEL_CIRCUMFERENCE = CHOSEN_MODULE.wheelCircumference;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
            new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
            new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
            new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0));

        /* Module Gear Ratios */
        public static final double DRIVE_GEAR_RATIO = CHOSEN_MODULE.driveGearRatio;
        public static final double ANGLE_GEAR_RATIO = CHOSEN_MODULE.angleGearRatio;

        /* Motor Inverts */
        public static final boolean INVERT_ANGLE_MOTORS = CHOSEN_MODULE.angleMotorInvert;
        public static final boolean INVERT_DRIVE_MOTORS = CHOSEN_MODULE.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue INVERT_CAN_CODER = SensorDirectionValue.Clockwise_Positive;
        // public static final SensorDirectionValue INVERT_CAN_CODER = CHOSEN_MODULE.canCoderInvert;

        /* Swerve Current Limiting */
        public static final int ANGLE_CONTINUOUS_CURRENT_LIMIT = 25;
        public static final int ANGLE_PEAK_CURRENT_LIMIT = 40;
        public static final double ANGLE_PEAK_CURRENT_DURATION = 0.1;
        public static final boolean ANGLE_ENABLE_CURRENT_LIMIT = true;

        public static final int DRIVE_CONTINUOUS_CURRENT_LIMIT = 35;
        public static final int DRIVE_PEAK_CURRENT_LIMIT = 60;
        public static final double DRIVE_PEAK_CURRENT_DURATION = 0.1;
        public static final boolean DRIVE_ENABLE_CURRENT_LIMIT = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double OPEN_LOOP_RAMP = 0.25;
        public static final double CLOSED_LOOP_RAMP = 0.0;

        /* Angle Motor PID Values */
        public static final double ANGLE_P = CHOSEN_MODULE.angleKP;
        public static final double ANGLE_I = CHOSEN_MODULE.angleKI;
        public static final double ANGLE_D = CHOSEN_MODULE.angleKD;
        public static final double ANGLE_F = CHOSEN_MODULE.angleKF;

        /* Drive Motor PID Values */
        public static final double DRIVE_P = 0.05; //TODO: This must be tuned to specific robot
        public static final double DRIVE_I = 0.0;
        public static final double DRIVE_D = 0.0;
        public static final double DRIVE_F = 0.0;

        /* Drive Motor Characterization Values 
         * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
        /** Percent Output */
        public static final double DRIVE_S = (0.13126 / 12); //TODO: This must be tuned to specific robot
        /** Percent Output */
        public static final double DRIVE_V = (2.6745 / 12);
        /** Percent Output */
        public static final double DRIVE_A = (0.24541 / 12);

        /* Swerve Profiling Values */
        /** Meters per second */
        public static final double MAX_SPEED = 2; //TODO: This must be tuned to specific robot
        /** Radians per second */
        public static final double MAX_ANGULAR_SPEED = 4; //TODO: This must be tuned to specific robot

        /* Neutral Modes and Idle Modes */
        public static final NeutralModeValue ANGLE_NEUTRAL_MODE = NeutralModeValue.Brake;
        public static final NeutralModeValue DRIVE_NEUTRAL_MODE = NeutralModeValue.Brake;
        public static final IdleMode ANGLE_IDLE_MODE = IdleMode.kBrake;
        public static final IdleMode DRIVE_IDLE_MODE = IdleMode.kBrake;

        public static final CTREConfigs CTRE_CONFIGS = new CTREConfigs();

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { //TODO: This must be tuned to specific robot
            public static final int DRIVE_MOTOR_ID = 1;
            public static final int ANGLE_MOTOR_ID = 2;
            public static final int CAN_CODER_ID = 11;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(64.775390625);
            public static final SwerveModuleConstants MODULE = 
                new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CAN_CODER_ID, ANGLE_OFFSET);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { //TODO: This must be tuned to specific robot
            public static final int DRIVE_MOTOR_ID = 3;
            public static final int ANGLE_MOTOR_ID = 4;
            public static final int CAN_CODER_ID = 12;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(218.232421875);
            public static final SwerveModuleConstants MODULE = 
                new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CAN_CODER_ID, ANGLE_OFFSET);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 { //TODO: This must be tuned to specific robot
            public static final int DRIVE_MOTOR_ID = 5;
            public static final int ANGLE_MOTOR_ID = 6;
            public static final int CAN_CODER_ID = 13;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(283.798828125);
            public static final SwerveModuleConstants MODULE = 
                new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CAN_CODER_ID, ANGLE_OFFSET);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { //TODO: This must be tuned to specific robot
            public static final int DRIVE_MOTOR_ID = 7;
            public static final int ANGLE_MOTOR_ID = 8;
            public static final int CAN_CODER_ID = 14;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(88.154296875);
            public static final SwerveModuleConstants MODULE = 
                new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CAN_CODER_ID, ANGLE_OFFSET);
        }
    }

    public static final class AutoConstants { //TODO: The below constants are used in the example auto, and must be tuned to specific robot
        /** Meters per second */
        public static final double MAX_SPEED = 9;
        /** Meters per second squared */
        public static final double MAX_ACCELERATION = 2;
        /** Radians per second */
        public static final double MAX_ANGULAR_SPEED = Math.PI;
        /** Radians per second squared */
        public static final double MAX_ANGULAR_ACCELERATION = Math.PI;
    
        public static final double DRIVE_X_P = 2.5;
        public static final double DRIVE_Y_P = 2.5;
        public static final double DRIVE_THETA_P = 4;
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints DRIVE_THETA_CONSTRAINTS =
            new TrapezoidProfile.Constraints(
                MAX_ANGULAR_SPEED, MAX_ANGULAR_ACCELERATION);
    }
}
