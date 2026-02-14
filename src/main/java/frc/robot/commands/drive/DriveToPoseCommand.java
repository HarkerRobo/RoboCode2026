package frc.robot.commands.drive;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AlignConstants;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.*;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class DriveToPoseCommand {

    private CommandSwerveDrivetrain drivetrain;
    private Pose2d targetPose;
    private Command pathCommand;
    private double aprilTagId;

    /** Updates the target pose dynamically based on AprilTag ID */
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

    private Pose2d getTargetPose(int tagId) {
        return switch (tagId) {
            case 31, 15 -> AlignConstants.CLIMB_CENTER;
            case 32, 16 -> AlignConstants.CLIMB_RIGHT;
            default -> drivetrain.getState().Pose;
        };
    }
}

