package frc.team2412.swervedrivebase;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
    private RobotContainer robotContainer;
    private Controls controls;

    @Override
    public void robotInit() {
        robotContainer = new RobotContainer();
        // updateManager = new UpdateManager(
        //         robotContainer.drivetrainSubsystem
        // );
        // updateManager.startLoop(5.0e-3);
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    // public SimplePathBuilder builder;
    // @Override
    // public void autonomousInit() {
    //     robotContainer.drivetrainSubsystem.resetPose(RigidTransform2.ZERO);
    //     // SimplePathBuilder builder = new SimplePathBuilder(new Vector2(0, 0), Rotation2.fromDegrees(0));
    //     // robotContainer.drivetrainSubsystem.follow(builder.lineTo(new Vector2(20, 0)).lineTo(new Vector2(20, 20)).lineTo(new Vector2(0, 20)).lineTo(new Vector2(0, 0)).build());
    //     new TimedTaxiAutoCommand(robotContainer.drivetrainSubsystem).schedule();
    // }

    @Override
    public void testInit() {
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void disabledPeriodic() {
        // robotContainer.drivetrainSubsystem.drive(0.0, 0.0, Rotation2d.fromDegrees(0), false);
    }

    @Override
    public void teleopInit() {
        controls = new Controls(robotContainer);
    }

    @Override
    public void teleopPeriodic() {
    }
}
