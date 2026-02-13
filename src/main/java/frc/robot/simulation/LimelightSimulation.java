package frc.robot.simulation;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

/**
 * A class to simulate a Limelight camera in simulation mode
 */
public class LimelightSimulation {
    // The network table used by the simulated Limelight
    private final NetworkTable limelightTable;

    // The simulated camera position relative to robot center
    private Transform3d robotToCamera;

    // Field visualization
    private final Field2d field2d;

    // AprilTag field layout
    private AprilTagFieldLayout aprilTagFieldLayout;

    // Maximum distance at which tags can be detected
    private static final double MAX_VISION_RANGE_METERS = 8.0;

    // Field of view limits (in degrees)
    private static final double HORIZONTAL_FOV_DEG = 60.0;
    private static final double VERTICAL_FOV_DEG = 45.0;

    // Last robot pose for visualization
    private Pose3d lastRobotPose;

    /**
     * Creates a new LimelightSimulation
     * @param limelightName The name of the Limelight ("limelight", "limelight-two", etc.)
     * @param robotToCamera The transform from robot center to camera
     */
    public LimelightSimulation(String limelightName, Transform3d robotToCamera) {
        this.limelightTable = NetworkTableInstance.getDefault().getTable(limelightName);
        this.robotToCamera = robotToCamera;
        this.field2d = new Field2d();
        SmartDashboard.putData("Field " + limelightName, field2d);

        // Load the AprilTag field layout
        try {
            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2025ReefscapeWelded.m_resourceFile);
        } catch (IOException e) {
            // Fall back to the default field
            try {
                aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.kDefaultField.m_resourceFile);
            } catch (IOException ex) {
                System.err.println("Failed to load AprilTag field layout: " + ex.getMessage());
                aprilTagFieldLayout = new AprilTagFieldLayout(new ArrayList<>(), 16.5, 8.0);
            }
        }
        
        // Initialize NetworkTable entries
        limelightTable.getEntry("tv").setDouble(0.0);
        limelightTable.getEntry("tx").setDouble(0.0);
        limelightTable.getEntry("ty").setDouble(0.0);
        limelightTable.getEntry("ta").setDouble(0.0);
        limelightTable.getEntry("tid").setDouble(-1);
    }

    /**
     * Update the simulated Limelight based on current robot pose
     * @param robotPose Current robot pose in field coordinates
     */
    public void update(Pose2d robotPose) {
        // Convert 2d robot pose to 3d
        Pose3d robotPose3d = new Pose3d(
                robotPose.getX(),
                robotPose.getY(),
                0.0,
                new Rotation3d(0, 0, robotPose.getRotation().getRadians()));
        
        lastRobotPose = robotPose3d;

        // Get camera pose in field coordinates
        Pose3d cameraPose = robotPose3d.transformBy(robotToCamera);

        // Find visible AprilTags
        List<Integer> visibleTagIds = findVisibleTags(cameraPose);

        // Update NetworkTables with simulated values
        limelightTable.getEntry("tv").setDouble(visibleTagIds.isEmpty() ? 0.0 : 1.0);

        if (!visibleTagIds.isEmpty()) {
            // Use the closest visible tag
            int closestTagId = visibleTagIds.get(0);
            
            // Get the tag pose
            Optional<Pose3d> tagPoseOptional = aprilTagFieldLayout.getTagPose(closestTagId);
            if (tagPoseOptional.isEmpty()) {
                // No tag pose available
                limelightTable.getEntry("tv").setDouble(0.0);
                return;
            }
            
            Pose3d tagPose = tagPoseOptional.get();
            
            // Calculate transform from camera to tag
            Transform3d cameraToTag = new Transform3d(cameraPose, tagPose);
            
            // Calculate angles from camera to tag
            double distance = cameraToTag.getTranslation().getNorm();
            double tx = Units.radiansToDegrees(Math.atan2(cameraToTag.getY(), cameraToTag.getX()));
            double ty = Units.radiansToDegrees(Math.atan2(cameraToTag.getZ(), 
                    Math.sqrt(cameraToTag.getX() * cameraToTag.getX() + cameraToTag.getY() * cameraToTag.getY())));
            
            limelightTable.getEntry("tx").setDouble(tx);
            limelightTable.getEntry("ty").setDouble(ty);
            limelightTable.getEntry("ta").setDouble(calculateTargetArea(distance));
            limelightTable.getEntry("tid").setDouble(closestTagId);

            // Update pose estimation
            double[] botpose = new double[6];
            botpose[0] = robotPose.getX();
            botpose[1] = robotPose.getY();
            botpose[2] = 0; // Z position
            botpose[3] = 0; // Roll
            botpose[4] = 0; // Pitch
            botpose[5] = robotPose.getRotation().getDegrees(); // Yaw
            
            limelightTable.getEntry("botpose").setDoubleArray(botpose);
            limelightTable.getEntry("botpose_wpiblue").setDoubleArray(botpose);
            
            // Target pose in camera space - inverse of camera to tag transform
            Transform3d tagToCamera = cameraToTag.inverse();
            
            double[] targetpose = new double[6];
            targetpose[0] = tagToCamera.getX();
            targetpose[1] = tagToCamera.getY();
            targetpose[2] = tagToCamera.getZ();
            targetpose[3] = Units.radiansToDegrees(tagToCamera.getRotation().getX());
            targetpose[4] = Units.radiansToDegrees(tagToCamera.getRotation().getY());
            targetpose[5] = Units.radiansToDegrees(tagToCamera.getRotation().getZ());
            
            limelightTable.getEntry("targetpose_cameraspace").setDoubleArray(targetpose);
            
            // Robot to camera transform
            double[] camerapose = new double[6];
            camerapose[0] = robotToCamera.getX();
            camerapose[1] = robotToCamera.getY();
            camerapose[2] = robotToCamera.getZ();
            camerapose[3] = Units.radiansToDegrees(robotToCamera.getRotation().getX());
            camerapose[4] = Units.radiansToDegrees(robotToCamera.getRotation().getY());
            camerapose[5] = Units.radiansToDegrees(robotToCamera.getRotation().getZ());
            
            limelightTable.getEntry("camerapose_robotspace").setDoubleArray(camerapose);
        } else {
            // No targets visible - clear values
            limelightTable.getEntry("tx").setDouble(0.0);
            limelightTable.getEntry("ty").setDouble(0.0);
            limelightTable.getEntry("ta").setDouble(0.0);
            limelightTable.getEntry("tid").setDouble(-1);
        }

        // Update field visualization
        updateField2d();
    }

    /**
     * Find AprilTag IDs that are visible from the camera pose
     * @param cameraPose Camera pose in field coordinates
     * @return List of visible AprilTag IDs
     */
    private List<Integer> findVisibleTags(Pose3d cameraPose) {
        List<Integer> visibleTagIds = new ArrayList<>();
        
        // Check all tags in the field layout
        for (AprilTag tag : aprilTagFieldLayout.getTags()) {
            Optional<Pose3d> tagPoseOptional = aprilTagFieldLayout.getTagPose(tag.ID);
            if (tagPoseOptional.isEmpty()) {
                continue;
            }
            
            Pose3d tagPose = tagPoseOptional.get();
            
            // Vector from camera to tag
            Translation3d cameraToTag = tagPose.getTranslation().minus(cameraPose.getTranslation());
            
            // Distance to tag
            double distance = cameraToTag.getNorm();
            
            // Skip if too far
            if (distance > MAX_VISION_RANGE_METERS) {
                continue;
            }
            
            // Get camera's forward vector
            Translation3d cameraForward = new Translation3d(1, 0, 0).rotateBy(cameraPose.getRotation());
            
            // Normalize camera-to-tag vector
            Translation3d cameraToTagNorm = cameraToTag.div(distance);
            
            // Calculate dot product to get cosine of angle between vectors
            double dot = cameraForward.getX() * cameraToTagNorm.getX() +
                         cameraForward.getY() * cameraToTagNorm.getY() +
                         cameraForward.getZ() * cameraToTagNorm.getZ();
            
            // Calculate angle in degrees
            double angleRad = Math.acos(Math.min(1.0, Math.max(-1.0, dot)));
            double angleDeg = Math.toDegrees(angleRad);
            
            // Tag is visible if within FOV
            if (angleDeg <= HORIZONTAL_FOV_DEG / 2 && angleDeg <= VERTICAL_FOV_DEG / 2) {
                // Check if tag is facing the camera
                // Get tag's orientation
                Rotation3d tagRotation = tagPose.getRotation();
                
                // Tag's forward direction (normal to tag surface)
                Translation3d tagNormal = new Translation3d(1, 0, 0).rotateBy(tagRotation);
                
                // Vector from tag to camera
                Translation3d tagToCamera = cameraPose.getTranslation().minus(tagPose.getTranslation());
                double tagToCameraDist = tagToCamera.getNorm();
                
                // Normalize
                Translation3d tagToCameraNorm = tagToCamera.div(tagToCameraDist);
                
                // Dot product should be negative if tag is facing camera
                double tagDot = tagNormal.getX() * tagToCameraNorm.getX() +
                                tagNormal.getY() * tagToCameraNorm.getY() +
                                tagNormal.getZ() * tagToCameraNorm.getZ();
                
                if (tagDot < 0) {
                    visibleTagIds.add(tag.ID);
                }
            }
        }

        // Sort by distance
        visibleTagIds.sort((a, b) -> {
            Optional<Pose3d> poseA = aprilTagFieldLayout.getTagPose(a);
            Optional<Pose3d> poseB = aprilTagFieldLayout.getTagPose(b);
            
            if (poseA.isEmpty() || poseB.isEmpty()) {
                return 0;
            }
            
            double distA = poseA.get().getTranslation().getDistance(cameraPose.getTranslation());
            double distB = poseB.get().getTranslation().getDistance(cameraPose.getTranslation());
            
            return Double.compare(distA, distB);
        });

        return visibleTagIds;
    }

    /**
     * Calculate the target area based on distance to target
     * @param distance Distance to target in meters
     * @return Target area (0-100)
     */
    private double calculateTargetArea(double distance) {
        // Simple model: target area is inversely proportional to square of distance
        return Math.min(20.0, 100.0 / (distance * distance));
    }

    /**
     * Updates the transform from robot to camera
     * @param robotToCamera New transform from robot center to camera
     */
    public void setRobotToCamera(Transform3d robotToCamera) {
        this.robotToCamera = robotToCamera;
    }

    /**
     * Updates the field visualization for this Limelight
     */
    private void updateField2d() {
        // Update AprilTag poses on field visualization
        for (AprilTag tag : aprilTagFieldLayout.getTags()) {
            Optional<Pose3d> tagPoseOptional = aprilTagFieldLayout.getTagPose(tag.ID);
            if (tagPoseOptional.isPresent()) {
                Pose3d tagPose = tagPoseOptional.get();
                field2d.getObject("Tag " + tag.ID)
                        .setPose(new Pose2d(
                                tagPose.getX(),
                                tagPose.getY(),
                                new Rotation2d(tagPose.getRotation().getZ())));
            }
        }
        
        // Show camera position and FOV if we have a robot pose
        if (lastRobotPose != null) {
            // Camera pose in field coordinates
            Pose3d cameraPose = lastRobotPose.transformBy(robotToCamera);
            
            // Camera position
            field2d.getObject("Camera")
                    .setPose(new Pose2d(
                            cameraPose.getX(),
                            cameraPose.getY(),
                            new Rotation2d(cameraPose.getRotation().getZ())));
            
            // FOV lines
            double fovLength = MAX_VISION_RANGE_METERS;
            double cameraHeading = cameraPose.getRotation().getZ();
            double leftAngle = cameraHeading - Math.toRadians(HORIZONTAL_FOV_DEG / 2);
            double rightAngle = cameraHeading + Math.toRadians(HORIZONTAL_FOV_DEG / 2);
            
            // Left FOV line
            field2d.getObject("FOV Left")
                    .setPose(new Pose2d(
                            cameraPose.getX() + Math.cos(leftAngle) * fovLength,
                            cameraPose.getY() + Math.sin(leftAngle) * fovLength,
                            new Rotation2d(leftAngle)));
            
            // Right FOV line
            field2d.getObject("FOV Right")
                    .setPose(new Pose2d(
                            cameraPose.getX() + Math.cos(rightAngle) * fovLength,
                            cameraPose.getY() + Math.sin(rightAngle) * fovLength,
                            new Rotation2d(rightAngle)));
            
            // Show visible tags
            List<Integer> visibleTagIds = findVisibleTags(cameraPose);
            for (int i = 0; i < visibleTagIds.size(); i++) {
                Optional<Pose3d> tagPoseOptional = aprilTagFieldLayout.getTagPose(visibleTagIds.get(i));
                if (tagPoseOptional.isPresent()) {
                    Pose3d tagPose = tagPoseOptional.get();
                    field2d.getObject("Visible " + i)
                            .setPose(new Pose2d(
                                    tagPose.getX(),
                                    tagPose.getY(),
                                    new Rotation2d(tagPose.getRotation().getZ())));
                }
            }
        }
    }

    public Field2d getField2d() {
        return field2d;
    }
}