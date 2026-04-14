package frc.robot.commands.drive;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Swerve;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.Robot;
import frc.robot.Telemetry;
import edu.wpi.first.wpilibj.DriverStation;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.util.FlippingUtil;
import com.ctre.phoenix6.swerve.SwerveRequest;

import frc.robot.Constants;

public class RotateToAngle extends Command
{
    CommandSwerveDrivetrain dt;

    private double MaxSpeed = 1.0 * Swerve.SPEED_AT_12_VOLTS.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
    private Supplier<Translation2d> targetSupplier;
    private Translation2d target;
    private boolean continueDrive;

    /**
     * Creates a rotation‑to‑target command for the swerve drivetrain.
     * Stores the target supplier and whether translational driving continues.
     */
    public RotateToAngle(CommandSwerveDrivetrain drivetrain, Supplier<Translation2d> targetSupplier,
        boolean continueDrive) 
    {
        dt = drivetrain;
        addRequirements(dt);
        this.targetSupplier = targetSupplier;
        this.continueDrive = continueDrive;
    }
    /**
     * Nothing
     */
    @Override
    public void initialize() 
    {
    }

    /**
     * Computes the desired heading angle toward the target point.
     * Applies alliance‑based flipping to maintain correct field orientation.
     */
    private Rotation2d calcAngle() 
    {
        double xdiff = target.getX() - dt.getState().Pose.getX();
        double ydiff = target.getY() - dt.getState().Pose.getY();
        double angle = Math.atan2(ydiff, xdiff);// + Math.PI;
        // if (xdiff < 0) angle = Math.PI + angle;

        if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Red) == DriverStation.Alliance.Red)
        {
            angle = Math.PI + angle;
        }
        return new Rotation2d(angle);
    }

    /**
     * Updates the target, computes the heading, and applies rotation control.
     * Optionally preserves driver translational input depending on configuration.
     */
    @Override
    public void execute() 
    {
        Translation2d rawTarget = targetSupplier.get();
        boolean red = DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
        if (red) target = FlippingUtil.flipFieldPosition(rawTarget);
        else target = rawTarget;

        double xSpeed = -Robot.instance.robotContainer.driver.getLeftY() * MaxSpeed;
        double ySpeed = -Robot.instance.robotContainer.driver.getLeftX() * MaxSpeed;
        SwerveRequest.FieldCentricFacingAngle drive = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        if (continueDrive)
        {
            dt.setControl(drive
                    .withHeadingPID(Constants.Swerve.AUTOALIGN_STEER_KP, Constants.Swerve.AUTOALIGN_STEER_KP,
                            Constants.Swerve.AUTOALIGN_STEER_KD)
                    .withTargetDirection(calcAngle())
                    .withMaxAbsRotationalRate(MaxAngularRate)
                    .withVelocityX(xSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(ySpeed) // Drive left with negative X (left)
            );
        }
        else
        {
            dt.setControl(drive
                    .withHeadingPID(Constants.Swerve.AUTOALIGN_STEER_KP, Constants.Swerve.AUTOALIGN_STEER_KP,
                            Constants.Swerve.AUTOALIGN_STEER_KD)
                    .withTargetDirection(calcAngle())
                    .withMaxAbsRotationalRate(MaxAngularRate)
                    .withVelocityX(0.0)
                    .withVelocityY(0.0)
                    // .withVelocityX(xSpeed * MaxSpeed)
                    // .withVelocityY(ySpeed * MaxSpeed)
            );
        }
        
        System.out.println("Rotating to target...");
        System.out.println("Current angle: " + dt.getState().Pose.getRotation());
        Telemetry.getInstance().test1.accept(dt.getState().Pose.getRotation().getDegrees());
        System.out.println("Target rotation: " + (calcAngle().getDegrees()));
        Telemetry.getInstance().test2.accept(calcAngle().getDegrees());
        System.out.println("ERROR: " + (dt.getState().Pose.getRotation().getDegrees() + 180.0 - calcAngle().getDegrees()));
        System.out.println("X- and Y- Speeds: " + (xSpeed * MaxSpeed) + ", " + (ySpeed * MaxSpeed));
    }

    /**
     * Determines whether rotation has reached the target heading.
     * Always returns false when translational driving is enabled.
     */
    @Override
    public boolean isFinished() 
    {
        if (continueDrive) return false;

        double angle = dt.getState().Pose.getRotation().getDegrees() /*+ 180.0*/ - calcAngle().getDegrees();
        angle = (angle + 360.0) % 360.0;

        if (Math.min(Math.abs(angle), Math.abs(angle - 360.0)) < 1.0)
        {
            Telemetry.getInstance().aligned.set(true);
            return true;
        }
        Telemetry.getInstance().aligned.set(false);
        return false;
    }

    /**
     * Nothing.
     */
    @Override
    public void end (boolean interrupted)
    {
        // SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
        // dt.setControl(brake);
    }
}
