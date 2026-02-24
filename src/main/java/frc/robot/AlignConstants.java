package frc.robot;

import edu.wpi.first.math.geometry.*;

public class AlignConstants {
    public static final Translation2d HUB = new Translation2d(4.606, 4.044);
    //public static final Translation2d HUB_RED = new Translation2d(11.979, 4.044);


    public static final Pose2d CLIMB = new Pose2d(0, 0, new Rotation2d(0)); //TODO (apriltag)
    //public static final Pose2d CLIMB_RIGHT = new Pose2d(0, 0, new Rotation2d(0));

    public static final Transform2d LEFT_OFFSET = new Transform2d(0, 0, new Rotation2d()); //TODO
    public static final Transform2d RIGHT_OFFSET = new Transform2d(0, 0, new Rotation2d());
    public static final Transform2d CENTER_OFFSET = new Transform2d(0, 0, new Rotation2d());

}
