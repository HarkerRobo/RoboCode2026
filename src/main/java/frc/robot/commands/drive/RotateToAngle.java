package frc.robot.commands.drive;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TunerConstants;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.Robot;
import frc.robot.Telemetry;
import edu.wpi.first.wpilibj.DriverStation;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.util.FlippingUtil;
import com.ctre.phoenix6.swerve.SwerveRequest;

import frc.robot.Constants;

public class RotateToAngle extends Command{
    CommandSwerveDrivetrain dt;

    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
    private Supplier<Translation2d> targetSupplier;
    private Translation2d target;

    public RotateToAngle(CommandSwerveDrivetrain drivetrain, Supplier<Translation2d> targetSupplier) {
        dt = drivetrain;
        addRequirements(dt);
        this.targetSupplier = targetSupplier;
    }

    /**
     * Reads and flips the target position based on alliance color. 
     * Stores the final field‑relative target for angle calculation.
     */
    @Override
    public void initialize() 
    {
        Translation2d rawTarget = targetSupplier.get();
        boolean red = DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
        if (red) target = FlippingUtil.flipFieldPosition(rawTarget);
        else target = rawTarget;
    }

    /**
     * Computes the field‑relative angle from the robot to the target. 
     * Returns a Rotation2d representing the desired facing direction.
     */
    private Rotation2d calcAngle() {
        double xdiff = target.getX() - dt.getState().Pose.getX();
        double ydiff = target.getY() - dt.getState().Pose.getY();
        double angle = Math.atan(ydiff/xdiff) + Math.PI;
        if (xdiff < 0) angle = Math.PI + angle;
        return new Rotation2d(angle);
    }

    /**
     * Commands the drivetrain to rotate toward the computed target angle. 
     * Publishes telemetry and prints diagnostic information during alignment.
     */
    @Override
    public void execute() {
        
        double xSpeed = -Robot.instance.robotContainer.driver.getLeftY();
        double ySpeed = -Robot.instance.robotContainer.driver.getLeftX();

        SwerveRequest.FieldCentricFacingAngle drive = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        dt.setControl(drive
            .withHeadingPID(Constants.Drive.autoalignSteerKP, Constants.Drive.autoalignSteerKI, Constants.Drive.autoalignSteerKD)
            .withTargetDirection(calcAngle())
            .withMaxAbsRotationalRate(MaxAngularRate)
            .withVelocityX(0.0)
            .withVelocityY(0.0)
            /*.withVelocityX(xSpeed * MaxSpeed)
            .withVelocityY(ySpeed * MaxSpeed)*/);
        
        System.out.println("Rotating to target...");
        System.out.println("Current angle: " + dt.getState().Pose.getRotation());
        Telemetry.getInstance().test1.accept(dt.getState().Pose.getRotation().getDegrees());
        System.out.println("Target rotation: " + (calcAngle().getDegrees()));
        Telemetry.getInstance().test2.accept(calcAngle().getDegrees());
        System.out.println("ERROR: " + (calcAngle().getDegrees() - dt.getState().Pose.getRotation().getDegrees()));
        System.out.println("X- and Y- Speeds: " + (xSpeed * MaxSpeed) + ", " + (ySpeed * MaxSpeed));
    }

    /**
     * Finishes when the robot’s heading is within tolerance of the target angle. 
     * Updates alignment telemetry to reflect whether the robot is facing correctly.
     */
    @Override
    public boolean isFinished() {
        if ((dt.getState().Pose.getRotation().getDegrees() + 180 - calcAngle().getDegrees()) % 360 <= 1.0)
        {
            Telemetry.getInstance().aligned.set(true);
            return true;
        }
        Telemetry.getInstance().aligned.set(false);
        return false;
    }

    /**
     * Applies a braking request when the command ends. 
     * Ensures the drivetrain stops rotating after alignment completes.
     */
    @Override
    public void end (boolean interrupted)
    {
        SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
        dt.applyRequest(()->brake);
    }
}
