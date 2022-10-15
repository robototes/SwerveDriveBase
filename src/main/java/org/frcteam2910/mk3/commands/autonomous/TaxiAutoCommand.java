package org.frcteam2910.mk3.commands.autonomous;

import org.frcteam2910.common.control.SimplePathBuilder;
import org.frcteam2910.common.control.Trajectory;
import org.frcteam2910.common.math.RigidTransform2;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.mk3.subsystems.DrivetrainSubsystem;

public class TaxiAutoCommand extends Follow2910TrajectoryCommand {
    public static final Trajectory robotPath = new Trajectory(
        new SimplePathBuilder(new Vector2(397.308, 122.461), Rotation2.fromDegrees(316.877))
                .lineTo(new Vector2(429.597, 88.203), Rotation2.fromDegrees(315))
                .lineTo(new Vector2(426.841, 94.110), Rotation2.fromDegrees(234))
                .build(),
        DrivetrainSubsystem.CONSTRAINTS, 0.1);

    public static final RigidTransform2 startPose = new RigidTransform2(new Vector2(8.922690, 5.802328), Rotation2.fromDegrees(41.745938));

    private final DrivetrainSubsystem drivebaseSubsystem;

    public TaxiAutoCommand(DrivetrainSubsystem drivebaseSubsystem) {
        super(drivebaseSubsystem, robotPath);
        this.drivebaseSubsystem = drivebaseSubsystem;
    }

    @Override
    public void initialize() {
        super.initialize();
        drivebaseSubsystem.resetPose(startPose);
    }
}
