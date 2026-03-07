package frc.robot.commands.drive;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AlignConstants;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.AlignDirection;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;

public class DriveToPose extends Command{
    private final CommandSwerveDrivetrain drivetrain;
    private Pose2d targetPose;
    private Command pathCommand;

    public DriveToPose(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        AlignDirection direction = Robot.instance.robotContainer.getAlignDirection();
        System.out.println("Starting DriveToPoseCommand...");
        updateTargetPose();
        startPath();
    }

    @Override
    public void execute() {
        if (pathCommand != null) {
            pathCommand.execute(); 
        }

        SmartDashboard.putString("Drive/direction", Robot.instance.robotContainer.getAlignDirection().toString());
    }

    @Override
    public boolean isFinished() {
        return pathCommand == null || pathCommand.isFinished();
    }

    /** Updates the target pose dynamically based on AprilTag ID */
    private void updateTargetPose() {

        if (targetPose != null && drivetrain.getState().Pose.equals(targetPose)) {
            System.out.println("Already at target pose. No path needed.");
            pathCommand = null; 
            return;
        }

        targetPose = getTargetPose();
        
        // Flip pose if we're on the red alliance
        if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) targetPose = FlippingUtil.flipFieldPose(targetPose);
        
    }

    private void startPath() {
        if (targetPose != null) {
            Pose2d currentPose = drivetrain.getState().Pose;
            
            // Check if the robot is already at the target position within a small tolerance
            double distance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
            double angleDifference = Math.abs(currentPose.getRotation().getDegrees() - targetPose.getRotation().getDegrees());
    
            if (distance < 0.01 && angleDifference < 2) { // 1 cm and 2 degrees tolerance
                System.out.println("Already at target pose. No path needed.");
                pathCommand = null;
                return;
            }
            
            List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(currentPose, targetPose);
    
            if (waypoints.isEmpty()) {
                System.out.println("No waypoints generated. Skipping path.");
                pathCommand = null;
                return;
            }

            PathPlannerPath generatedPath = new PathPlannerPath(waypoints, 
            Constants.Vision.constraints, null, 
            new GoalEndState(0, targetPose.getRotation()));
            generatedPath.preventFlipping = true;
            pathCommand = AutoBuilder.followPath(generatedPath);

            if (pathCommand == null) {
                System.out.println("PathPlanner failed to generate a command. Skipping execution.");
                return;
            }
    
            try {
                pathCommand.initialize();
            } catch (Exception e) {
                e.printStackTrace();
                return;
            }
        }
    }

    private Pose2d getTargetPose() {

        return switch (Robot.instance.robotContainer.getAlignDirection()) {
            case Center -> AlignConstants.CLIMB_CENTER;
            case Right -> AlignConstants.CLIMB_CENTER.plus(AlignConstants.LEFT_OFFSET);
            case Left -> AlignConstants.CLIMB_CENTER.plus(AlignConstants.RIGHT_OFFSET);
            default -> drivetrain.getState().Pose;
        };
    }
}
