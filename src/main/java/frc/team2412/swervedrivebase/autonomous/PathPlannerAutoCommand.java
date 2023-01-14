package frc.team2412.swervedrivebase.autonomous;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2412.swervedrivebase.RobotContainer;
import frc.team2412.swervedrivebase.subsystems.DrivetrainSubsystem;

public class PathPlannerAutoCommand extends CommandBase {
    
    private DrivetrainSubsystem drivetrainSubsystem;
    private RobotContainer r;

    public PathPlannerAutoCommand(DrivetrainSubsystem drivetrainSubsystem, RobotContainer r) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.r = r;
    }

    @Override
    public void initialize() {
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("AutoTestPath", new PathConstraints(0.2, 0.1));
        r.autoBuilder.fullAuto(pathGroup).schedule();
    }

}
