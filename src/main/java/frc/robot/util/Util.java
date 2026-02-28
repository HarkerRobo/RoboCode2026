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

import com.pathplanner.lib.util.FlippingUtil;

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
    
    public static Angle calculatePitch(Translation3d position, Translation3d target, double shootVelocity)
    {
        double dx = target.getX() - position.getX();
        double dy = target.getY() - position.getY();
        double db = Math.sqrt(dx * dx + dy * dy);
        double dz = target.getZ() - position.getZ();
        double s = shootVelocity;
        double g = Constants.G;
        System.out.println("\n\tdb: " + db + "\tdz: " + dz + "\ts: " + s);
        double idealAngle = Math.atan((Math.pow(s,2.0) + 
            Math.sqrt(Math.pow(s,4.0)
                - g * (g * Math.pow(db,2.0) + 2.0 * dz * Math.pow(s,2.0))))
            / (g * db));
        return Radians.of(idealAngle);
    }

    public static double calculateVelocity(Translation3d position, Translation3d target)
    {
        double dx = target.getX() - position.getX();
        double dy = target.getY() - position.getY();
        double db = Math.sqrt(dx * dx + dy * dy);
        return Constants.DISTANCE_SHOOTVELO_RATIO * (db - Constants.Simulation.HUB_CONTENTS.getXWidth() / 2.0 - 6.0) + 10.0;
    }

    public static Angle calculateShootPitch(CommandSwerveDrivetrain drivetrain)
    {
        
        return Util.calculatePitch(new Translation3d(
                drivetrain.getState().Pose.getTranslation().getX(),
                drivetrain.getState().Pose.getTranslation().getY(),
                Constants.HOOD_BASE_HEIGHT), 
                (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) ? 
                    Constants.HUB_TARGET_POSITION : 
                    new Translation3d(FlippingUtil.flipFieldPosition(Constants.HUB_TARGET_POSITION.toTranslation2d()).getX(), FlippingUtil.flipFieldPosition(Constants.HUB_TARGET_POSITION.toTranslation2d()).getY(), Constants.HUB_TARGET_POSITION.getZ()), 
                calculateShootVelocity(drivetrain));
    }

    public static double calculateShootVelocity(CommandSwerveDrivetrain drivetrain)
    {
        return Util.calculateVelocity(new Translation3d(
                drivetrain.getState().Pose.getTranslation().getX(),
                drivetrain.getState().Pose.getTranslation().getY(),
                Constants.HOOD_BASE_HEIGHT),
                (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) ? 
                    Constants.HUB_TARGET_POSITION : 
                    new Translation3d(FlippingUtil.flipFieldPosition(Constants.HUB_TARGET_POSITION.toTranslation2d()).getX(), FlippingUtil.flipFieldPosition(Constants.HUB_TARGET_POSITION.toTranslation2d()).getY(), Constants.HUB_TARGET_POSITION.getZ()));
    }
    
    public static Angle calculatePassPitch(CommandSwerveDrivetrain drivetrain)
    {
        return Util.calculatePitch(new Translation3d(
                drivetrain.getState().Pose.getTranslation().getX(),
                drivetrain.getState().Pose.getTranslation().getY(),
                Constants.HOOD_BASE_HEIGHT),
                (drivetrain.getState().Pose.getY() < Constants.Simulation.FIELD_HEIGHT / 2.0) ?
                    (DriverStation.getAlliance().orElse(Alliance.Blue)==Alliance.Blue) ? Constants.PASS_LEFT_TARGET_POSITION : new Translation3d(FlippingUtil.flipFieldPosition(Constants.PASS_LEFT_TARGET_POSITION.toTranslation2d()).getX(), FlippingUtil.flipFieldPosition(Constants.PASS_LEFT_TARGET_POSITION.toTranslation2d()).getY(), Constants.PASS_LEFT_TARGET_POSITION.getZ()) :
                    (DriverStation.getAlliance().orElse(Alliance.Blue)==Alliance.Blue) ? Constants.PASS_RIGHT_TARGET_POSITION : new Translation3d(FlippingUtil.flipFieldPosition(Constants.PASS_RIGHT_TARGET_POSITION.toTranslation2d()).getX(), FlippingUtil.flipFieldPosition(Constants.PASS_RIGHT_TARGET_POSITION.toTranslation2d()).getY(), Constants.PASS_RIGHT_TARGET_POSITION.getZ()) ,
                calculatePassVelocity(drivetrain));
    }
    
    public static double calculatePassVelocity(CommandSwerveDrivetrain drivetrain)
    {
        return Util.calculateVelocity(new Translation3d(
                drivetrain.getState().Pose.getTranslation().getX(),
                drivetrain.getState().Pose.getTranslation().getY(),
                Constants.HOOD_BASE_HEIGHT),
                (drivetrain.getState().Pose.getY() < Constants.Simulation.FIELD_HEIGHT / 2.0) ?
                    (DriverStation.getAlliance().orElse(Alliance.Blue)==Alliance.Blue) ? Constants.PASS_LEFT_TARGET_POSITION : new Translation3d(FlippingUtil.flipFieldPosition(Constants.PASS_LEFT_TARGET_POSITION.toTranslation2d()).getX(), FlippingUtil.flipFieldPosition(Constants.PASS_LEFT_TARGET_POSITION.toTranslation2d()).getY(), Constants.PASS_LEFT_TARGET_POSITION.getZ()) :
                    (DriverStation.getAlliance().orElse(Alliance.Blue)==Alliance.Blue) ? Constants.PASS_RIGHT_TARGET_POSITION : new Translation3d(FlippingUtil.flipFieldPosition(Constants.PASS_RIGHT_TARGET_POSITION.toTranslation2d()).getX(), FlippingUtil.flipFieldPosition(Constants.PASS_RIGHT_TARGET_POSITION.toTranslation2d()).getY(), Constants.PASS_RIGHT_TARGET_POSITION.getZ()));
    }

    public static double bound(double value, double min, double max)
    {
        if (value < min) return min;
        if (value > max) return max;
        return value;
    }

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