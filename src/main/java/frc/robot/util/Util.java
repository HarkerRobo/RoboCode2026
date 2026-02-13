package frc.robot.util;


import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.math.geometry.Translation2d;

import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Util 
{
    public static double bottomLeftX (Rectangle2d r)
    {
        return r.getCenter().getX() - 0.5 * r.getXWidth();
    }
    
    public static double bottomLeftY (Rectangle2d r)
    {
        return r.getCenter().getY() - 0.5 * r.getYWidth();
    }
    
    public static double topRightX (Rectangle2d r)
    {
        return r.getCenter().getX() + 0.5 * r.getXWidth();
    }
    
    public static double topRightY (Rectangle2d r)
    {
        return r.getCenter().getY() + 0.5 * r.getYWidth();
    }

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

    public static boolean within (Rectangle2d r, Translation3d t)
    {
        return 
            t.getX() > r.getCenter().getX() - 0.5 * r.getXWidth() && t.getX() < r.getCenter().getX() + 0.5 * r.getXWidth() &&
            t.getY() > r.getCenter().getY() - 0.5 * r.getYWidth() && t.getY() < r.getCenter().getY() + 0.5 * r.getYWidth();
    }

    public static Rectangle2d rotate(Rectangle2d r)
    {
        return new Rectangle2d(new Pose2d(new Translation2d(Constants.Simulation.ROTATE_X.apply(r.getCenter().getX()), Constants.Simulation.ROTATE_Y.apply(r.getCenter().getY())), new Rotation2d()), r.getXWidth(), r.getYWidth());
    }
    
    public static Angle calculatePitch(Translation3d position, Translation3d target)
    {
        double dx = target.getX() - position.getX();
        double dy = target.getY() - position.getY();
        double base = Math.sqrt(dx * dx + dy * dy);
        double height = target.getZ() - position.getZ();
        double idealAngle = Math.atan2(height, base);
        return Radians.of(idealAngle + 10.0); // TODO make this real
    }

    public static double calculateVelocity(Translation3d position, Translation3d target)
    {
        return 10.0; // TODO make this real
    }

    public static Angle calculateShootPitch(CommandSwerveDrivetrain drivetrain)
    {
        return Util.calculatePitch(new Translation3d(
                drivetrain.getState().Pose.getTranslation().getX(),
                drivetrain.getState().Pose.getTranslation().getY(),
                Constants.HOOD_BASE_HEIGHT), Constants.HUB_TARGET_POSITION);
    }

    public static double calculateShootVelocity(CommandSwerveDrivetrain drivetrain)
    {
        return Util.calculateVelocity(new Translation3d(
                drivetrain.getState().Pose.getTranslation().getX(),
                drivetrain.getState().Pose.getTranslation().getY(),
                Constants.HOOD_BASE_HEIGHT), Constants.HUB_TARGET_POSITION);
    }
    
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

    public static double bound(double value, double min, double max)
    {
        if (value < min) return min;
        if (value > max) return max;
        return value;
    }

    public static boolean onLeftSide(CommandSwerveDrivetrain drivetrain)
    {
        return drivetrain.getState().Pose.getY() < Constants.Simulation.FIELD_HEIGHT;
    }

}