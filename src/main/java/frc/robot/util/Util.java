package frc.robot.util;


import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants;

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
}