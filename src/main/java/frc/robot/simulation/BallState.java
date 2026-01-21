package frc.robot.simulation;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * FRC 868 code
 */

public class BallState 
{
    public Pose3d pose;
    public Translation3d velocity;
    public Translation3d omega; // rad/s

    public BallState(
            Pose3d position,
            Translation3d velocity,
            Translation3d omega) 
    {
        this.pose = position;
        this.velocity = velocity;
        this.omega = omega;
    }
}
