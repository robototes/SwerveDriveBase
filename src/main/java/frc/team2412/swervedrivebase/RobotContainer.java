package frc.team2412.swervedrivebase;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team2412.swervedrivebase.subsystems.*;

public class RobotContainer {

    public final DrivetrainSubsystem drivetrainSubsystem;
    public final SwerveAutoBuilder autoBuilder;

    public RobotContainer(){
        drivetrainSubsystem = new DrivetrainSubsystem();

        autoBuilder = new SwerveAutoBuilder(
            drivetrainSubsystem::getPose, 
            drivetrainSubsystem::resetPose,
            drivetrainSubsystem.getKinematics(),
            new PIDConstants(0.0708, 0, 0), // translation PID
            new PIDConstants(5.0, 0, 0), // rot PID
            drivetrainSubsystem::drive,
            new HashMap<String, Command>(),
            drivetrainSubsystem
        );
    }

}
