package frc.robot.commands.drive;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class DriveToPose extends Command {
    
    CommandSwerveDrivetrain drive;

    public DriveToPose(CommandSwerveDrivetrain drivetrain) 
    {
        drive = drivetrain;
        addRequirements(drive);
    }

    private Rotation2d calcAngle() {
        double deltaY = Constants.Drivetrain.HUB.getY() - drive.getState().Pose.getY();
        double deltaX = Constants.Drivetrain.HUB.getX() - drive.getState().Pose.getX();
        double angle = Math.atan(deltaY/deltaX);
        return new Rotation2d(angle);
    }

    @Override
    public void initialize()
    {
    }

    @Override
    public void execute() 
    {
        SwerveRequest.FieldCentricFacingAngle d = new SwerveRequest.FieldCentricFacingAngle()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
        drive.setControl(d.withTargetDirection(calcAngle()));
    }

    @Override
    public boolean isFinished()
    {
        //checks if you are facing target rotation
        return drive.getState().Pose.getRotation().equals(calcAngle());
    }

    @Override
    public void end(boolean interrupted) {
        
    }
}