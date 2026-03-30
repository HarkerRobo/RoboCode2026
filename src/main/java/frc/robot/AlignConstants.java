package frc.robot;
import static edu.wpi.first.units.Units.*;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.*;

public class AlignConstants {
    public static final Translation2d HUB = new Translation2d(4.611624, 4.021328);
    //public static final Translation2d HUB_RED = new Translation2d(11.979, 4.044);


    public static final Pose2d CLIMB_LEFT = new Pose2d(1.0556, 3.7455 + 0.89535, new Rotation2d(Degrees.of(-90))); //left from driver station
    public static final Pose2d CLIMB_RIGHT = new Pose2d(1.0556, 3.7455 - 0.89535, new Rotation2d(Degrees.of(90))); 
    //public static final Pose2d CLIMB_RIGHT = new Pose2d(0, 0, new Rotation2d(0));

    //public static final Transform2d RIGHT_OFFSET = new Transform2d(0, -0.1524, new Rotation2d(0)); 
    //public static final Transform2d LEFT_OFFSET = new Transform2d(0, 0.1524, new Rotation2d(0));
}