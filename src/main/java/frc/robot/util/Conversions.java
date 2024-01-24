package frc.robot.util;

public class Conversions {

    public static double rotationsToDegrees(double rotations) {
        return rotations * 360;
    }

    public static double degreesToRotations(double degrees) {
        return degrees / 360;
    }

    public static double RPSToRPM(double RPS) {
        return RPS * 60;
    }

    public static double RPMToRPS(double RPM) {
        return RPM / 60;
    }

    public static double RPSToMPS(double RPS, double circumference) {
        return RPS * circumference;
    }

    public static double MPSToRPS(double MPS, double circumference) {
        return MPS / circumference;
    }

    public static double rotationsToMeters(double rotations, double circumference) {
        return rotations * circumference;
    }

    public static double metersToRotations(double meters, double circumference) {
        return meters / circumference;
    }
}