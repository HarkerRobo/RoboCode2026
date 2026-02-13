package frc.robot.commands.drive;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.wpilibj.DriverStation;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import frc.robot.AlignConstants;

public class DriveToPose extends Command{
    CommandSwerveDrivetrain dt;

    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    public DriveToPose(CommandSwerveDrivetrain drivetrain) {
        dt = drivetrain;
        addRequirements(dt);
    }

    public void init() {
    }

    private Rotation2d calcAngle() {
        boolean red = DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
        double xdiff = (red ? AlignConstants.HUB_RED.getX() : AlignConstants.HUB_BLUE.getX()) - dt.getState().Pose.getX();
        double ydiff = (red ? AlignConstants.HUB_RED.getY() : AlignConstants.HUB_BLUE.getY()) - dt.getState().Pose.getY();
        double angle = Math.atan(ydiff/xdiff);
        if (xdiff < 0) angle = Math.PI +angle;
        return new Rotation2d(angle);
    }

    public void execute() {
        
        double xSpeed = -RobotContainer.getInstance().getJoystickLeftY();
        double ySpeed = -RobotContainer.getInstance().getJoystickLeftX();

        SwerveRequest.FieldCentricFacingAngle drive = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        dt.setControl(drive
            .withHeadingPID(10, 0, 0.1)
            .withTargetDirection(calcAngle())
            .withMaxAbsRotationalRate(MaxAngularRate)
            .withVelocityX(xSpeed * MaxSpeed)
            .withVelocityY(ySpeed * MaxSpeed));
        
        System.out.println("Rotating to hub...");
        System.out.println("Current angle: " + dt.getState().Pose.getRotation());
        System.out.println("Target rotation: " + calcAngle());
        System.out.println("X- and Y- Speeds: " + (xSpeed * MaxSpeed) + ", " + (ySpeed * MaxSpeed));
    }

    public boolean isFinished() {
        return (Math.abs(dt.getState().Pose.getRotation().getRadians() - calcAngle().getRadians()) <= 0.01);
    }
}
