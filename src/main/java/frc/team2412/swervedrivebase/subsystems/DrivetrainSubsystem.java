package frc.team2412.swervedrivebase.subsystems;

import frc.team2412.swervedrivebase.Constants;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DrivetrainSubsystem extends SubsystemBase {

    private static final double ticksPerRotation = 2048.0;
    private static final double wheelDiameterMeters = 0.0762; // 3 inches
    private static final double driveReductionL1 = 8.14; // verified
    private static final double steerReduction = (32.0 / 15.0) * (60.0 / 10.0); // verified, 12.8

    private static final int talonFXLoopNumber = 0;
    private static final double maxSteeringSpeed = 1.0;

    private Field2d field = new Field2d();

    // position units is one rotation / 2048
    // extrapolate this to meters using wheel perimeter (pi * wheel diameter)
    // raw sensor unit per meter driven = ticks/ perimeter

    // units: raw sensor units
    private static final double steerPositionCoefficient = (ticksPerRotation/(2*Math.PI)) * steerReduction; // radians per tick
    private static final double driveVelocityCoefficient = (ticksPerRotation / (Math.PI * wheelDiameterMeters)) * driveReductionL1; // ticks per meter per 100 ms

    private WPI_TalonFX[] moduleDriveMotors = {
        new WPI_TalonFX(Constants.DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR),
        new WPI_TalonFX(Constants.DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR),
        new WPI_TalonFX(Constants.DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR),
        new WPI_TalonFX(Constants.DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR)
    };

    private WPI_TalonFX[] moduleAngleMotors = {
        new WPI_TalonFX(Constants.DRIVETRAIN_FRONT_LEFT_ANGLE_MOTOR),
        new WPI_TalonFX(Constants.DRIVETRAIN_FRONT_RIGHT_ANGLE_MOTOR),
        new WPI_TalonFX(Constants.DRIVETRAIN_BACK_LEFT_ANGLE_MOTOR),
        new WPI_TalonFX(Constants.DRIVETRAIN_BACK_RIGHT_ANGLE_MOTOR)
    };

    private WPI_CANCoder[] moduleEncoders = {
        new WPI_CANCoder(Constants.DRIVETRAIN_FRONT_LEFT_ENCODER_PORT),
        new WPI_CANCoder(Constants.DRIVETRAIN_FRONT_RIGHT_ENCODER_PORT),
        new WPI_CANCoder(Constants.DRIVETRAIN_BACK_LEFT_ENCODER_PORT),
        new WPI_CANCoder(Constants.DRIVETRAIN_BACK_RIGHT_ENCODER_PORT)
    };

    private Rotation2d[] moduleOffsets = {
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

    private AHRS gyroscope;

    private SwerveDriveOdometry odometry;
    private Pose2d pose;

    public DrivetrainSubsystem() {
        gyroscope = new AHRS(SerialPort.Port.kUSB1);

        odometry = new SwerveDriveOdometry(kinematics, gyroscope.getRotation2d(), getModulePositions());
        pose = odometry.getPoseMeters();

        // configure encoders offsets
        for (int i = 0; i < moduleEncoders.length; i++) {
            moduleEncoders[i].configFactoryDefault();
        }

        // configure drive motors
        for (int i = 0; i < moduleDriveMotors.length; i++) {
            WPI_TalonFX driveMotor = moduleDriveMotors[i];
            driveMotor.setNeutralMode(NeutralMode.Brake);
            driveMotor.setSensorPhase(true);
            
            driveMotor.configNeutralDeadband(0.01);
            driveMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, talonFXLoopNumber, Constants.CAN_TIMEOUT_MS);

            driveMotor.config_kP(talonFXLoopNumber, 0.1);
            driveMotor.config_kI(talonFXLoopNumber, 0.001);
            driveMotor.config_kD(talonFXLoopNumber, 1023.0/20660.0);
        }

        // configure angle motors
        for (int i = 0; i < moduleAngleMotors.length; i++) {
            WPI_TalonFX steeringMotor = moduleAngleMotors[i];
            steeringMotor.configFactoryDefault();
            steeringMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, talonFXLoopNumber, Constants.CAN_TIMEOUT_MS);
            // Make the integrated encoder count forever (don't wrap), since it doesn't work properly with continuous mode
            // We account for this manually (unfortunately)
            // steeringMotor.configFeedbackNotContinuous(true, Constants.CAN_TIMEOUT_MS);
            // Configure PID values
            steeringMotor.config_kP(talonFXLoopNumber, 0.15, Constants.CAN_TIMEOUT_MS);
            steeringMotor.config_kI(talonFXLoopNumber, 0.00, Constants.CAN_TIMEOUT_MS);
            steeringMotor.config_kD(talonFXLoopNumber, 1.0, Constants.CAN_TIMEOUT_MS);
            // Limit steering module speed
            // steeringMotor.configPeakOutputForward(maxSteeringSpeed, Constants.CAN_TIMEOUT_MS);
            // steeringMotor.configPeakOutputReverse(-maxSteeringSpeed, Constants.CAN_TIMEOUT_MS);

            steeringMotor.setSelectedSensorPosition((moduleEncoders[i].getAbsolutePosition() - moduleOffsets[i].getDegrees()) * ((ticksPerRotation/360) * steerReduction));
        }

        // configure shuffleboard

        SmartDashboard.putData("Field", field);
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
            moduleStates = getModuleStates(
                ChassisSpeeds.fromFieldRelativeSpeeds(forward, -strafe, rotation.getRadians()*100, getGyroRotation2d())
            );
        } else {
            moduleStates = getModuleStates(new ChassisSpeeds(forward, -strafe, rotation.getRadians()*100));
        }
        drive(moduleStates);
    } 

    public void drive(ChassisSpeeds chassisSpeeds) {
        SwerveModuleState[] moduleStates = getModuleStates(chassisSpeeds);
        drive(moduleStates);
    }

    public void zeroGyroAngle() {
        gyroscope.setAngleAdjustment(gyroscope.getAngle());
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
            // System.out.println(states[i].speedMetersPerSecond);
            moduleDriveMotors[i].set(TalonFXControlMode.Velocity, ((states[i].speedMetersPerSecond)/10) * driveVelocityCoefficient);
            //System.out.println((states[i].speedMetersPerSecond/10) * driveVelocityCoefficient);
        }
        for (int i=0; i < moduleAngleMotors.length; i++) {
            moduleAngleMotors[i].set(TalonFXControlMode.Position, states[i].angle.getRadians() * steerPositionCoefficient); // steerpositioncoefficient is maybe fixed
            // moduleAngleMotors[i].set(TalonFXControlMode.Position, 1000);
            // System.out.println(states[i].angle.getRadians() * steerPositionCoefficient);
            // System.out.println("Module number " + i + " has encoder position: " + moduleEncoders[i].getAbsolutePosition() + " and sensor position: " + moduleAngleMotors[i].getSelectedSensorPosition() * ((360/ticksPerRotation) * steerReduction));
            // System.out.println("Module number" + i + " position: " + moduleEncoders[i].getAbsolutePosition());
        }
    }

    /**
     * @param speeds
     * @return Array with modules with front left at [0], front right at [1], back left at [2], back right at [3]
     */
    public SwerveModuleState[] getModuleStates(ChassisSpeeds speeds) {
        return kinematics.toSwerveModuleStates(speeds);
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            new SwerveModulePosition(moduleDriveMotors[0].getSelectedSensorPosition(), 
                new Rotation2d(moduleEncoders[0].getAbsolutePosition() - moduleOffsets[0].getDegrees())),
            new SwerveModulePosition(moduleDriveMotors[1].getSelectedSensorPosition(), 
                new Rotation2d(moduleEncoders[1].getAbsolutePosition() - moduleOffsets[1].getDegrees())),
            new SwerveModulePosition(moduleDriveMotors[2].getSelectedSensorPosition(), 
                new Rotation2d(moduleEncoders[2].getAbsolutePosition() - moduleOffsets[2].getDegrees())),
            new SwerveModulePosition(moduleDriveMotors[3].getSelectedSensorPosition(), 
                new Rotation2d(moduleEncoders[3].getAbsolutePosition() - moduleOffsets[3].getDegrees())),
        };
    }

    public Pose2d getPose() {
        return pose;
    }

    public void resetPose(Pose2d pose) {
        odometry.resetPosition(pose.getRotation(), getModulePositions(), pose);
        this.pose = pose;
    }

    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    public Rotation2d getGyroRotation2d() {
        return Rotation2d.fromDegrees(-gyroscope.getAngle());
    }

    @Override
    public void periodic() {
        pose = odometry.update(getGyroRotation2d(), getModulePositions());
        field.setRobotPose(pose);
    }

}