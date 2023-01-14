package org.frcteam2910.mk3.subsystems;

import org.frcteam2910.mk3.Constants;
import org.opencv.calib3d.StereoBM;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.AbsoluteEncoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.frcteam2910.common.robot.UpdateManager;

public class DrivetrainSubsystem extends SubsystemBase implements UpdateManager.Updatable {

    private final double ticksPerRotation = 2048.0;
    private final double wheelDiameterMeters = 0.0762; // 3 inches
    private final double driveReductionL3 = (14.0 / 50.0) * (28.0 / 16.0) * (15.0 / 45.0); // verified
    private final double steerReduction = (32.0 / 15.0) * (60.0 / 10.0); // verified, 12.8

    private static final int TALONFX_PID_LOOP_NUMBER = 0;
    private static final double MAX_STEERING_SPEED = 1.0;

    // position units is one rotation / 2048
    // extrapolate this to meters using wheel perimeter (pi * wheel diameter)
    // raw sensor unit = perimeter / 2048

    // units: raw sensor units
    // 32000 ticks per rotation
    // 16000 ticks per pi radians
    private final double steerPositionCoefficient = (ticksPerRotation/(2*Math.PI)) * steerReduction;
    private final double driveVelocityCoefficient = (Math.PI * wheelDiameterMeters * driveReductionL3 * ticksPerRotation);

    WPI_TalonFX[] moduleDriveMotors = {
        new WPI_TalonFX(Constants.DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR),
        new WPI_TalonFX(Constants.DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR),
        new WPI_TalonFX(Constants.DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR),
        new WPI_TalonFX(Constants.DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR)
    };

    WPI_TalonFX[] moduleAngleMotors = {
        new WPI_TalonFX(Constants.DRIVETRAIN_FRONT_LEFT_ANGLE_MOTOR),
        new WPI_TalonFX(Constants.DRIVETRAIN_FRONT_RIGHT_ANGLE_MOTOR),
        new WPI_TalonFX(Constants.DRIVETRAIN_BACK_LEFT_ANGLE_MOTOR),
        new WPI_TalonFX(Constants.DRIVETRAIN_BACK_RIGHT_ANGLE_MOTOR)
    };

    WPI_CANCoder[] moduleEncoders = {
        new WPI_CANCoder(Constants.DRIVETRAIN_FRONT_LEFT_ENCODER_PORT),
        new WPI_CANCoder(Constants.DRIVETRAIN_FRONT_RIGHT_ENCODER_PORT),
        new WPI_CANCoder(Constants.DRIVETRAIN_BACK_LEFT_ENCODER_PORT),
        new WPI_CANCoder(Constants.DRIVETRAIN_BACK_RIGHT_ENCODER_PORT)
    };

    double[] moduleOffsets = {
        Constants.DRIVETRAIN_FRONT_LEFT_ENCODER_OFFSET,
        Constants.DRIVETRAIN_FRONT_RIGHT_ENCODER_OFFSET,
        Constants.DRIVETRAIN_BACK_LEFT_ENCODER_OFFSET,
        Constants.DRIVETRAIN_BACK_RIGHT_ENCODER_OFFSET
    };

    // 2ft x 2ft for practice bot
    private final Translation2d[] moduleLocations = {
        new Translation2d(Units.feetToMeters(1), Units.feetToMeters(1)), // front left
        new Translation2d(Units.feetToMeters(1), Units.feetToMeters(-1)), // front right
        new Translation2d(Units.feetToMeters(-1), Units.feetToMeters(1)), // back left
        new Translation2d(Units.feetToMeters(-1), Units.feetToMeters(-1)) // back right
    };

    SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        moduleLocations[0], moduleLocations[1], moduleLocations[2], moduleLocations[3]
    );

    public DrivetrainSubsystem() {
        // configure encoders offsets
        for (int i = 0; i < moduleEncoders.length; i++) {
            moduleEncoders[i].configMagnetOffset(moduleOffsets[i]);
        }

        // configure drive motors
        for (int i = 0; i < moduleDriveMotors.length; i++) {
            WPI_TalonFX driveMotor = moduleDriveMotors[i];
            driveMotor.setNeutralMode(NeutralMode.Brake);
            driveMotor.setSensorPhase(true);
        }

        // configure angle motors
        for (int i = 0; i < moduleAngleMotors.length; i++) {
            WPI_TalonFX steeringMotor = moduleAngleMotors[i];
            steeringMotor.configFactoryDefault();
            steeringMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, TALONFX_PID_LOOP_NUMBER, Constants.CAN_TIMEOUT_MS);
            // Make the integrated encoder count forever (don't wrap), since it doesn't work properly with continuous mode
            // We account for this manually (unfortunately)
            // steeringMotor.configFeedbackNotContinuous(true, Constants.CAN_TIMEOUT_MS);
            // Configure PID values
            steeringMotor.config_kP(TALONFX_PID_LOOP_NUMBER, 0.15, Constants.CAN_TIMEOUT_MS);
            steeringMotor.config_kI(TALONFX_PID_LOOP_NUMBER, 0.00, Constants.CAN_TIMEOUT_MS);
            steeringMotor.config_kD(TALONFX_PID_LOOP_NUMBER, 1.0, Constants.CAN_TIMEOUT_MS);
            // Limit steering module speed
            // steeringMotor.configPeakOutputForward(MAX_STEERING_SPEED, Constants.CAN_TIMEOUT_MS);
            // steeringMotor.configPeakOutputReverse(-MAX_STEERING_SPEED, Constants.CAN_TIMEOUT_MS);
        }
    }

    /**
     * Drives the robot using forward, strafe, and rotation. Units in meters
     * 
     * @param forward
     * @param strafe
     * @param rotation
     * @param fieldOriented
     */
    public void drive(double forward, double strafe, Rotation2d rotation, boolean fieldOriented) {
        SwerveModuleState[] moduleStates = getModuleStates(new ChassisSpeeds(0,0,0));
        if (fieldOriented) {
        
        } else {
            moduleStates = getModuleStates(new ChassisSpeeds(forward, strafe, rotation.getRadians()));
        }
        drive(moduleStates);
    } 

    /**
     * Drives the robot using states
     * 
     * @param states
     */
    public void drive(SwerveModuleState[] states) {     
        // Set motor speeds and angles
        for (int i=0; i < moduleDriveMotors.length; i++) {
            // meters/100ms * raw sensor units conversion
            // moduleDriveMotors[i].set(TalonFXControlMode.Velocity, (states[i].speedMetersPerSecond) * driveVelocityCoefficient * 10);
            // System.out.println((states[i].speedMetersPerSecond / 10) * driveVelocityCoefficient);
        }
        for (int i=0; i < moduleAngleMotors.length; i++) {
            moduleAngleMotors[i].set(TalonFXControlMode.Position, states[i].angle.getRadians() * steerPositionCoefficient); // steerpositioncoefficient is maybe fixed
            // moduleAngleMotors[i].set(TalonFXControlMode.Position, 1000);
            System.out.println(states[i].angle.getRadians() * steerPositionCoefficient);
        }

    }

    /**
     * @param speeds
     * @return Array with modules with front left at [0], front right at [1], back left at [2], back right at [3]
     */
    private SwerveModuleState[] getModuleStates(ChassisSpeeds speeds) {
        return kinematics.toSwerveModuleStates(speeds);
    }

    @Override
    public void update(double time, double dt) {
        // not sure team         
    }
}