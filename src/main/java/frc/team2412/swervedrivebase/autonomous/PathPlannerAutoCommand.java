package frc.team2412.swervedrivebase.autonomous;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2412.swervedrivebase.subsystems.DrivetrainSubsystem;

public class PathPlannerAutoCommand extends CommandBase {
    
    private DrivetrainSubsystem drivetrainSubsystem;

    public PathPlannerAutoCommand(DrivetrainSubsystem drivetrainSubsystem) {
        this.drivetrainSubsystem = drivetrainSubsystem;
    }

    @Override
    public void execute() {
        new PPSwerveControllerCommand(
            PathPlanner.loadPath("AutoTestPath", new PathConstraints(4, 3)),
            drivetrainSubsystem::getPose, 
            drivetrainSubsystem.getKinematics(),
            new PIDController(0, 0, 0),
            new PIDController(0, 0, 0),
            new PIDController(0, 0, 0),
            drivetrainSubsystem::drive,
            drivetrainSubsystem
        ).schedule();
    }

}
