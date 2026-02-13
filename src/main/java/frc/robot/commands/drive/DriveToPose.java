package frc.robot.commands.drive;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.TunerConstants;
import frc.robot.Robot;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.wpilibj.DriverStation;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.util.FlippingUtil;
import com.ctre.phoenix6.swerve.SwerveRequest;

import frc.robot.AlignConstants;

public class DriveToPose extends Command{
    CommandSwerveDrivetrain dt;

    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
    private Translation2d target;

    public DriveToPose(CommandSwerveDrivetrain drivetrain, Translation2d target) {
        dt = drivetrain;
        addRequirements(dt);
        this.target = target;
    }

    @Override
    public void initialize() 
    {
        boolean red = DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
        if (red) target = FlippingUtil.flipFieldPosition(target);
    }

    private Rotation2d calcAngle() {
        double xdiff = target.getX() - dt.getState().Pose.getX();
        double ydiff = target.getY() - dt.getState().Pose.getY();
        double angle = Math.atan(ydiff/xdiff) + Math.PI;
        if (xdiff < 0) angle = Math.PI + angle;
        return new Rotation2d(angle);
    }

    public void execute() {
        
        double xSpeed = -Robot.instance.robotContainer.driver.getLeftY();
        double ySpeed = -Robot.instance.robotContainer.driver.getLeftX();

        SwerveRequest.FieldCentricFacingAngle drive = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        dt.setControl(drive
            .withHeadingPID(10, 0, 0.1)
            .withTargetDirection(calcAngle())
            .withMaxAbsRotationalRate(MaxAngularRate)
            .withVelocityX(xSpeed * MaxSpeed)
            .withVelocityY(ySpeed * MaxSpeed));
        
        System.out.println("Rotating to target...");
        System.out.println("Current angle: " + dt.getState().Pose.getRotation());
        System.out.println("Target rotation: " + calcAngle());
        System.out.println("ERROR: " + (calcAngle().getDegrees() - dt.getState().Pose.getRotation().getDegrees()));
        System.out.println("X- and Y- Speeds: " + (xSpeed * MaxSpeed) + ", " + (ySpeed * MaxSpeed));
    }

    public boolean isFinished() {
        return (Math.abs(dt.getState().Pose.getRotation().getRadians() - calcAngle().getRadians()) <= 0.01);
    }
}
