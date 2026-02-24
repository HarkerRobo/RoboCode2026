package frc.robot.commands.drivetrain;

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
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class DriveToPose extends Command{
    private final CommandSwerveDrivetrain drivetrain;
    private Pose2d targetPose;
    private Command pathCommand;
    private double aprilTagId;

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
        SmartDashboard.putNumber("Drive/aprilTagId", aprilTagId);
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

    private Pose2d getTargetPose(int aprilTagId) {

        return switch (Robot.instance.robotContainer.getAlignDirection()) {
            case Center -> switch (aprilTagId) {
                case 15, 31 -> AlignConstants.CLIMB_CENTER;
                
                default -> {
                    System.out.println("Unknown AprilTag ID for center: " + aprilTagId);
                    yield drivetrain.getState().Pose;
                }
            };
            case Right -> switch (aprilTagId) {
                case 16, 32 -> AlignConstants.CLIMB_CENTER;
                
                default -> {
                    System.out.println("Unknown AprilTag ID for right: " + aprilTagId);
                    yield drivetrain.getState().Pose;
                }
            };
            default -> drivetrain.getState().Pose;
        };
    }
}
