package frc.team2412.swervedrivebase.autonomous;

import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2412.swervedrivebase.subsystems.DrivetrainSubsystem;

public class PathPlannerAutoCommand extends CommandBase {
    
    private DrivetrainSubsystem drivetrainSubsystem;

    public PathPlannerAutoCommand(DrivetrainSubsystem drivetrainSubsystem) {
        this.drivetrainSubsystem = drivetrainSubsystem;
    }

    @Override
    public void execute() {
        // new PPSwerveControllerCommand(
        //     trajectory,
        //     poseSupplier, 
        //     xController, 
        //     yController, 
        //     rotationController, 
        //     outputChassisSpeeds, 
        //     drivetrainSubsystem
        // ).schedule();
    }

}
