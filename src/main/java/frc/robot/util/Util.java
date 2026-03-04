package frc.robot.util;


import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.math.geometry.Translation2d;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * Utility helper methods used across robot code geometry, targeting, and 
 */
public class Util 
{
    /**
     * Finds the minimum x-coordinate of the rectangle
     * @param r the rectangle 
     * @return  the minimum x-coordinate
     */
    public static double bottomLeftX (Rectangle2d r)
    {
        return r.getCenter().getX() - 0.5 * r.getXWidth();
    }
    /**
     * Finds the minimum y-coordinate of the rectangle
     * @param r the rectangle
     * @return  the minimum y-coordinate
     */
    public static double bottomLeftY (Rectangle2d r)
    {
        return r.getCenter().getY() - 0.5 * r.getYWidth();
    }
    /**
     * Finds the maximum x-coordinate of the rectangle
     * @param r the rectangle
     * @return  the maximum x-coordinate
     */
    public static double topRightX (Rectangle2d r)
    {
        return r.getCenter().getX() + 0.5 * r.getXWidth();
    }
    /**
     * Find the maximum y-coordinate of the rectangle
     * @param r the rectangle
     * @return  the maximum y-coordinate
     */
    public static double topRightY (Rectangle2d r)
    {
        return r.getCenter().getY() + 0.5 * r.getYWidth();
    }
    /**
     * Computes the position of the num-th ball packed inside the rectangle footprint
     * @param r the rectangle footprint
     * @param bottomZ   the bottom Z coordinate of the first layer
     * @param sphereDiameter    the diameter of each ball
     * @param num   the index of the ball
     * @return  the positition of the num-th ball packed inside the rectangle footprint
     */
    public static Translation3d packEl (Rectangle2d r, double bottomZ, double sphereDiameter, int num)
    {
        double x0 = bottomLeftX(r);
        double y0 = bottomLeftY(r);
        double z0 = bottomZ;

        double dx = sphereDiameter;
        double dy = sphereDiameter;
        double dz = sphereDiameter;

        double Dx = r.getXWidth();
        double Dy = r.getYWidth();

        int xn = (int) (Dx / dx);
        int yn = (int) (Dy / dy);

        int zN = num / (xn * yn);
        if (zN != 0)
        {
            num %= zN * (xn * yn);
        }

        int yN = num / yn;
        if (yN != 0)
        {
            num %= yN * yn;
        }

        int xN = num;

        return new Translation3d(x0 + (xN + 1) * dx, y0 + (yN + 1) * dy, z0 + zN * dz);
    }
    /**
     * Determines whether a point lies within the x-y bounds of a rectangle
     * @param r the rectangle
     * @param t the point
     * @return  True if the point does lie within the x-y bounds of the rectangle; false otherwise
     */
    public static boolean within (Rectangle2d r, Translation3d t)
    {
        return 
            t.getX() > r.getCenter().getX() - 0.5 * r.getXWidth() && t.getX() < r.getCenter().getX() + 0.5 * r.getXWidth() &&
            t.getY() > r.getCenter().getY() - 0.5 * r.getYWidth() && t.getY() < r.getCenter().getY() + 0.5 * r.getYWidth();
    }
    /**
     * Rotates a rectangle's center
     * @param r the rectangle
     * @return  the new transformed rectangle
     */
    public static Rectangle2d rotate(Rectangle2d r)
    {
        return new Rectangle2d(new Pose2d(new Translation2d(Constants.Simulation.ROTATE_X.apply(r.getCenter().getX()), Constants.Simulation.ROTATE_Y.apply(r.getCenter().getY())), new Rotation2d()), r.getXWidth(), r.getYWidth());
    }
    /**
     * Calculates the launch pitch angle from a position to the target
     * @param position  the start position
     * @param target    the target position
     * @return  the calculated pitch angle
     */
    public static Angle calculatePitch(Translation3d position, Translation3d target)
    {
        double dx = target.getX() - position.getX();
        double dy = target.getY() - position.getY();
        double base = Math.sqrt(dx * dx + dy * dy);
        double height = target.getZ() - position.getZ();
        double idealAngle = Math.atan2(height, base);
        return Radians.of(idealAngle).plus(Degrees.of(10.0)); // TODO make this real
    }
    /**
     * Calculates the launch velocity from a position to the target
     * @param position  the start position
     * @param target    the target position
     * @return  the calculated launch velocity
     */
    public static double calculateVelocity(Translation3d position, Translation3d target)
    {
        return 10.0; // TODO make this real
    }
    /**
     * Calculates the shooting pitch angle based on the drivetrain pose and the hub target position
     * @param drivetrain    the drivetrain subsystem
     * @return  the calculated shooting pitch angle
     */
    public static Angle calculateShootPitch(CommandSwerveDrivetrain drivetrain)
    {
        return Util.calculatePitch(new Translation3d(
                drivetrain.getState().Pose.getTranslation().getX(),
                drivetrain.getState().Pose.getTranslation().getY(),
                Constants.HOOD_BASE_HEIGHT), Constants.HUB_TARGET_POSITION);
    }
    /**
     * Calculates the shooting pitch velocity based on the drivetrain pose and the hub target position
     * @param drivetrain    the drivetrain subsystem
     * @return  the calculated shooting pitch velocity
     */
    public static double calculateShootVelocity(CommandSwerveDrivetrain drivetrain)
    {
        return Util.calculateVelocity(new Translation3d(
                drivetrain.getState().Pose.getTranslation().getX(),
                drivetrain.getState().Pose.getTranslation().getY(),
                Constants.HOOD_BASE_HEIGHT), Constants.HUB_TARGET_POSITION);
    }
    /**
     * Calculates the passing pitch angle based on the drivetrain pose and the side of the pass target
     * @param drivetrain    the drivetrain subsystem
     * @return  the calculated passing pitch angle
     */
    public static Angle calculatePassPitch(CommandSwerveDrivetrain drivetrain)
    {
        return Util.calculatePitch(new Translation3d(
                drivetrain.getState().Pose.getTranslation().getX(),
                drivetrain.getState().Pose.getTranslation().getY(),
                Constants.HOOD_BASE_HEIGHT),
                (drivetrain.getState().Pose.getY() < Constants.Simulation.FIELD_HEIGHT / 2.0) ?
                    Constants.PASS_LEFT_TARGET_POSITION :
                    Constants.PASS_RIGHT_TARGET_POSITION);
    }
    /**
     * Calculates the passing velocity based on the drivetrain pose and the side of the passing target
     * @param drivetrain
     * @return
     */
    public static double calculatePassVelocity(CommandSwerveDrivetrain drivetrain)
    {
        return Util.calculateVelocity(new Translation3d(
                drivetrain.getState().Pose.getTranslation().getX(),
                drivetrain.getState().Pose.getTranslation().getY(),
                Constants.HOOD_BASE_HEIGHT),
                (drivetrain.getState().Pose.getY() < Constants.Simulation.FIELD_HEIGHT / 2.0) ?
                    Constants.PASS_LEFT_TARGET_POSITION :
                    Constants.PASS_RIGHT_TARGET_POSITION);
    }
    /**
     * Bounds a numeric value to a range
     * @param value the value being bounded
     * @param min   the minimum allowed value
     * @param max   the maximum allowed value
     * @return  the bounded value within [min, max]
     */
    public static double bound(double value, double min, double max)
    {
        if (value < min) return min;
        if (value > max) return max;
        return value;
    }
    /**
     * Determines whether the robot is on the alliance-relative left side of the field
     * @param drivetrain    the Drivetrain subsystem
     * @return  True if the robot is on the alliance-relative left side of the field; false otherwise
     */
    public static boolean onLeftSide(CommandSwerveDrivetrain drivetrain)
    {
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red)
        {
            
            return drivetrain.getState().Pose.getY() > Constants.Simulation.FIELD_HEIGHT / 2.0;
        }
        else
        {
            return drivetrain.getState().Pose.getY() < Constants.Simulation.FIELD_HEIGHT / 2.0;
        }
    }

}