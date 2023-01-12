package org.frcteam2910.mk3.commands.autonomous;

import org.frcteam2910.common.control.Trajectory;
import org.frcteam2910.mk3.subsystems.DrivetrainSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class Follow2910TrajectoryCommand extends CommandBase {
    private final DrivetrainSubsystem drivebase;
    private final Trajectory trajectory;

    public Follow2910TrajectoryCommand(DrivetrainSubsystem drivebase, Trajectory trajectory) {
        this.drivebase = drivebase;
        this.trajectory = trajectory;

        addRequirements(drivebase);
    }

    @Override
    public void initialize() {
        drivebase.getFollower().follow(trajectory);
    }

    @Override
    public void end(boolean interrupted) {
        drivebase.getFollower().cancel();
    }

    @Override
    public boolean isFinished() {
        return drivebase.getFollower().getCurrentTrajectory().isEmpty();
    }
}
