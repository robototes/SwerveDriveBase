package frc.team2412.swervedrivebase;

import edu.wpi.first.math.geometry.Rotation2d;

public class Constants {
    public static final int DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR = 1;
    public static final int DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR = 4;
    public static final int DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR = 7;
    public static final int DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR = 10;

    public static final int DRIVETRAIN_FRONT_LEFT_ANGLE_MOTOR = 2;
    public static final int DRIVETRAIN_FRONT_RIGHT_ANGLE_MOTOR = 5;
    public static final int DRIVETRAIN_BACK_LEFT_ANGLE_MOTOR = 8;
    public static final int DRIVETRAIN_BACK_RIGHT_ANGLE_MOTOR = 11;

    public static final int DRIVETRAIN_FRONT_LEFT_ENCODER_PORT = 3;
    public static final int DRIVETRAIN_FRONT_RIGHT_ENCODER_PORT = 6;
    public static final int DRIVETRAIN_BACK_LEFT_ENCODER_PORT = 9;
    public static final int DRIVETRAIN_BACK_RIGHT_ENCODER_PORT = 12;
    public static final int PIGEON_PORT = 20;

    // In degrees
    public static final Rotation2d DRIVETRAIN_FRONT_LEFT_ENCODER_OFFSET = Rotation2d.fromDegrees(0);
    public static final Rotation2d DRIVETRAIN_FRONT_RIGHT_ENCODER_OFFSET = Rotation2d.fromDegrees(0);
    public static final Rotation2d DRIVETRAIN_BACK_LEFT_ENCODER_OFFSET = Rotation2d.fromDegrees(0);
    public static final Rotation2d DRIVETRAIN_BACK_RIGHT_ENCODER_OFFSET = Rotation2d.fromDegrees(0);

    public static final int PRIMARY_CONTROLLER_PORT = 0;

    /**
     * How many milliseconds it takes for a CAN command to time out.
     */
    public static final int CAN_TIMEOUT_MS = 20;

}
