package frc.team2412.swervedrivebase.commands;

import java.util.function.DoubleSupplier;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2412.swervedrivebase.subsystems.DrivetrainSubsystem;

public class DriveCommand extends CommandBase {
//     private final DrivetrainSubsystem drivetrainSubsystem;
//     private final Axis forward;
//     private final Axis strafe;
//     private final Axis rotation;
//     private final boolean fieldOriented;

//     private static boolean USE_JOYSTICK = false;

//     public DriveCommand(DrivetrainSubsystem drivetrain, Axis forward, Axis strafe, Axis rotation, boolean fieldOriented) {
//         this.forward = forward;
//         this.strafe = strafe;
//         this.rotation = rotation;
//         this.fieldOriented = fieldOriented;

//         drivetrainSubsystem = drivetrain;

//         addRequirements(drivetrain);
//     }

//     @Override
//     public void execute() {
//         if (USE_JOYSTICK) {
//             Joystick joystick = Controls.
//             drivetrainSubsystem.drive(new Vector2(-joystick.getX(), joystick.getY()), Math.pow(joystick.getTwist(), 2)/3, fieldOriented);
//         } else {
//             drivetrainSubsystem.drive(new Vector2(-forward.get(false), strafe.get(false)), rotation.get(false), fieldOriented);
//         }
// //        if(drivetrainSubsystem.isFieldOriented()) System.out.println(drivetrainSubsystem.getGyroAngle());
//     }

//     @Override
//     public void end(boolean interrupted) {
//         drivetrainSubsystem.drive(Vector2.ZERO, 0, true);
//     }


    private final DrivetrainSubsystem drivebaseSubsystem;
    private final DoubleSupplier forward;
    private final DoubleSupplier strafe;
    private final DoubleSupplier rotation;

    private final AHRS gyro;

    public DriveCommand(DrivetrainSubsystem drivebaseSubsystem, DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier rotation) {
        this.drivebaseSubsystem = drivebaseSubsystem;
        this.forward = forward;
        this.strafe = strafe;
        this.rotation = rotation;
        this.gyro = new AHRS(SerialPort.Port.kMXP);

        addRequirements(drivebaseSubsystem);
    }

    @Override
    public void execute() {
        double x = deadbandCorrection(forward.getAsDouble());
        double y = deadbandCorrection(-strafe.getAsDouble());
        double rot = deadbandCorrection(-rotation.getAsDouble()) / 2;
        drivebaseSubsystem.drive(x, y, Rotation2d.fromDegrees(rot), true);

        // System.out.println("Angle: " + gyro.getAngle());
    }
 
    public double deadbandCorrection(double input) {
        return Math.abs(input) < 0.05 ? 0 : (input - Math.signum(input) * 0.05) / 0.95;
    }

}
