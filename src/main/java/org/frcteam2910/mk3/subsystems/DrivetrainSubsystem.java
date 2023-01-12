package org.frcteam2910.mk3.subsystems;

import org.frcteam2910.mk3.Constants;
import org.opencv.calib3d.StereoBM;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_CANCoder;

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
    private final double wheelDiameter = 3.0;
    private final double driveReduction = 1.0;
    private final double steerReduction = (32.0 / 15.0) * (60.0 / 10.0);

    private static final int TALONFX_PID_LOOP_NUMBER = 0;
    private static final double MAX_STEERING_SPEED = 0.5;

    // position units is one rotation / 2048
    // extrapolate this to meters using wheel perimeter (pi * wheel diameter)
    // raw sensor unit = perimeter / 2048

    // units: raw sensor units
    private final double steerPositionCoefficient = Math.PI * wheelDiameter * steerReduction / ticksPerRotation;
    private final double driveVelocityCoefficient = (Math.PI * wheelDiameter * driveReduction / ticksPerRotation) * 10.0;

    WPI_TalonFX[] moduleDriveMotors = {
        new WPI_TalonFX(Constants.DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR),
        new WPI_TalonFX(Constants.DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR),
        new WPI_TalonFX(Constants.DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR),
        new WPI_TalonFX(Constants.DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR)
    };

   // WPI_TalonFX motor1 = null;

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
        moduleEncoders[0].configMagnetOffset(moduleOffsets[0]);
        moduleEncoders[1].configMagnetOffset(moduleOffsets[1]);
        moduleEncoders[2].configMagnetOffset(moduleOffsets[2]);
        moduleEncoders[3].configMagnetOffset(moduleOffsets[3]); 

        for (int i = 0; i > moduleAngleMotors.length - 1; i++) {
            WPI_TalonFX steeringMotor = moduleAngleMotors[i];
            steeringMotor.configFactoryDefault();
            steeringMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, TALONFX_PID_LOOP_NUMBER, Constants.CAN_TIMEOUT_MS);
            // Make the integrated encoder count forever (don't wrap), since it doesn't work properly with continuous mode
            // We account for this manually (unfortunately)
            steeringMotor.configFeedbackNotContinuous(true, Constants.CAN_TIMEOUT_MS);
            // Configure PID values
            steeringMotor.config_kP(TALONFX_PID_LOOP_NUMBER, 0.05, Constants.CAN_TIMEOUT_MS);
            steeringMotor.config_kI(TALONFX_PID_LOOP_NUMBER, 0.01, Constants.CAN_TIMEOUT_MS);
            steeringMotor.config_kD(TALONFX_PID_LOOP_NUMBER, 0.0, Constants.CAN_TIMEOUT_MS);
            // Limit steering module speed
            steeringMotor.configPeakOutputForward(MAX_STEERING_SPEED, Constants.CAN_TIMEOUT_MS);
            steeringMotor.configPeakOutputReverse(-MAX_STEERING_SPEED, Constants.CAN_TIMEOUT_MS);
        }

        //motor1 = new WPI_TalonFX(Constants.DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR);
        //System.out.println(motor1.getSupplyCurrent());
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
            // moduleDriveMotors[i].set(TalonFXControlMode.Velocity, (states[i].speedMetersPerSecond / 10) * driveVelocityCoefficient);
            // System.out.println((states[i].speedMetersPerSecond / 10) * driveVelocityCoefficient);
        }
        for (int i=0; i < moduleAngleMotors.length; i++) {
            moduleAngleMotors[i].set(TalonFXControlMode.Position, states[i].angle.getRadians() * steerPositionCoefficient * 40000); // steerpositioncoefficient is WRONG
            // moduleAngleMotors[i].set(TalonFXControlMode.Position, 1000);
            // System.out.println(states[i].angle.getRadians() * steerPositionCoefficient);
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