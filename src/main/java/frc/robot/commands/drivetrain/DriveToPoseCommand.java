// code from team 5675

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AlignConstants;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.PassDirection;
//import frc.robot.subsystems.swerve.Drivetrain;
import edu.wpi.first.math.geometry.Pose2d;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.FlippingUtil;
import java.util.List;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveToPoseCommand extends Command {
    /*
    private final Drivetrain drivetrain;
    private Pose2d targetPose;
    private Command pathCommand;
    private boolean isBarge;
    private double aprilTagId;

    public DriveToPoseCommand(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
}

    @Override
    public void initialize() {
        AlignDirection direction = RobotContainer.getInstance().getAlignDirection();
        isBarge = direction == AlignDirection.MidBarge || direction == AlignDirection.LeftBarge || direction == AlignDirection.RightBarge;
        System.out.println("Starting DriveToPoseCommand...");
        updateTargetPose();
        startPath();
    }

    @Override
    public void execute() {
        if (pathCommand != null) {
            pathCommand.execute(); 
        }

        SmartDashboard.putString("Drive/direction", RobotContainer.getInstance().getAlignDirection().toString());
        SmartDashboard.putBoolean("Drive/isBarge", isBarge);
        SmartDashboard.putNumber("Drive/aprilTagId", aprilTagId);
    }

    @Override
    public boolean isFinished() {
        return pathCommand == null || pathCommand.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        if (pathCommand != null) {
            pathCommand.cancel();
            System.out.println("DriveToPoseCommand finished.");
            }
    }

    /** Updates the target pose dynamically based on AprilTag ID * /
    private void updateTargetPose() {

        if (targetPose != null && drivetrain.getState().Pose.equals(targetPose)) {
            System.out.println("Already at target pose. No path needed.");
            pathCommand = null; 
            return;
        }
        aprilTagId = LimelightHelpers.getFiducialID(Constants.Vision.kCamera1Name);
        
        if (aprilTagId == -1) {
            System.out.println("No valid AprilTag detected.");
            return;
        } else {
            targetPose = getTargetPose((int) aprilTagId);
        }

        // Flip pose if we're on the red alliance
        if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) targetPose = FlippingUtil.flipFieldPose(targetPose);
        
    }
    */
}