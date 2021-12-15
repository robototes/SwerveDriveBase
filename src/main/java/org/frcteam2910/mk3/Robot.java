package org.frcteam2910.mk3;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.frcteam2910.common.control.Trajectory;
import org.frcteam2910.common.robot.UpdateManager;

public class Robot extends TimedRobot {
    private RobotContainer robotContainer;
    private Controls controls;
    private UpdateManager updateManager;

    @Override
    public void robotInit() {
        robotContainer = new RobotContainer();
        controls = new Controls(robotContainer);
        updateManager = new UpdateManager(
                robotContainer.drivetrainSubsystem
        );
        updateManager.startLoop(5.0e-3);
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void autonomousInit() {
//        robotContainer.drivetrainSubsystem.follower.follow(new Trajectory());
    }

    @Override
    public void testInit() {
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void teleopInit() {
    }
}
