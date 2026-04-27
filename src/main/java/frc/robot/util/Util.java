package frc.robot.util;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import java.nio.file.WatchEvent;

import javax.naming.event.NamingListener;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.Constants;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;

public class Util 
{

    /**
     * Linearly interpolates between two velocity-angle pairs using the formula (1-t)*start + t*end.
     * A t value of 0 returns the start, 1 returns the end, and values between blend the two.
     */
    private static Interpolator<Pair<LinearVelocity, Angle>> interpolator = new Interpolator<>() {
        @Override
        public Pair<LinearVelocity, Angle> interpolate(Pair<LinearVelocity, Angle> startValue, 
                                                                         Pair<LinearVelocity, Angle> endValue, double t)
        {
            double v0 = startValue.getFirst().in(MetersPerSecond);
            double v1 = endValue.getFirst().in(MetersPerSecond);
            LinearVelocity vx = MetersPerSecond.of(v0 * (1.0 - t) + v1 * t);

            double th0 = startValue.getSecond().in(Rotations);
            double th1 = endValue.getSecond().in(Rotations);
            Angle thx = Rotations.of(th0 * (1.0 - t) + th1 * t);

            return Pair.of(vx, thx);
        }
    };

    private static InverseInterpolator<Double> inverseInterpolator = InverseInterpolator.forDouble();
    
    // angles, speeds are effective
    private static InterpolatingTreeMap<Double, Pair<LinearVelocity, Angle>> interpolatingTreeMap = new InterpolatingTreeMap<>(inverseInterpolator, interpolator);

    /**
     * Adds a data point to the interpolating tree map
     * 
     * @param distanceMeters          the distance in meters
     * @param velocityMetersPerSecond the shooting velocity in meters per second
     * @param angleDegrees            the shooting angle in degrees
     */
    private static void addData(double distanceMeters, double velocityMetersPerSecond, double angleDegrees)
    {
        interpolatingTreeMap.put(distanceMeters, Pair.of(MetersPerSecond.of(velocityMetersPerSecond), Degrees.of(angleDegrees)));
    }

    /**
     * Adds the shooting data points into the interpolating tree map
     */
    public static void init ()
    {
        // OLD DATA POINTS
        /*
        addData(1.891, 20.0, 75.0);
        addData(3.373, 19.0, 68.0);
        addData(3.703,19.0,66.0);
        */
        // addData(1.121, 12.5, 75.0); // old
        // addData(1.709, 18.5, 75.0); // old
        // addData(1.902, 19.0, 74.0);
        // //addData(2.483, 21.0, 73.0); // old
        // addData(2.678, 20.0, 72.0);
        // //addData(2.856, 21.0, 71.0); // old
        // addData(3.081, 20.0, 71.0);
        // addData(3.662, 19.0, 68.0);
        // addData(4.038, 19.0, 67.0);
        // addData(4.45, 19.0, 66.5);
        // addData(5.386, 21.0, 66.0);
        // 2.722, 70, 19.5
        
        // addData(4.646, 9.7, 69.7 + angleOffset); // "
        // addData(5.697, 10.2, 64.9 + angleOffset); // "; farthest point

        // 2026-03-28 DATA POINTS

        // double angleOffset = 0.0;
        // double velocityOffset = 0.5;//0.4;
        // addData(1.425, 7.4, 75.0 + angleOffset);
        // addData(1.864, 7.7, 71.5 + angleOffset);
        // addData(2.062, 8.0, 71.5 + angleOffset);
        // addData(2.320, 8.3, 71.5 + angleOffset);
        // addData(2.614, 8.4 + velocityOffset - 0.1, 71.0 + angleOffset);
        // addData(2.919, 8.5 + velocityOffset - 0.1, 70.8 + angleOffset);
        // addData(3.207, 8.35 + velocityOffset, 70.6 + angleOffset);
        // addData(3.40, 9.0 + velocityOffset, 70.3 + angleOffset);
        // addData(3.574, 8.8 + velocityOffset, 70.3 + angleOffset);
        // addData(3.80, 9.2 + velocityOffset, 70.1 + angleOffset);
        // addData(3.952, 9.26 + velocityOffset, 70.1 + angleOffset);
        // addData(4.0, 9.25 + velocityOffset, 70.0 + angleOffset);
        // addData(4.312, 9.35 + velocityOffset, 69.9 + angleOffset);
        // addData(5.20, 9.5 + velocityOffset, 67.5 + angleOffset);

        // NEW DATA POINTS FROM 4/8/26
        addData(1.265, 6.8, 75.0); // 1.366
        addData(1.360, 6.9, 75.0); // 1.454
        addData(1.574, 7.0, 74.0); // 1.574
        addData(1.487, 7.2, 73.0); // 1.736
        addData(1.838, 7.4, 72.0); // 1.909
        addData(1.961, 7.45, 71.5); // 2.027
        addData(2.120, 7.5, 70.5); // 2.182
        addData(2.333, 7.5, 69.5); // 2.389
        addData(2.525, 7.6, 68.5); // 2.577
        addData(2.750, 7.8, 67.5); // 2.798
        addData(3.020,8.0,66.0); // 3.064

    }

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
     * Computes the position of the num-th ball packed inside a rotated rectangular footprint
     * 
     * @param x              the center x-coordinate of the footprint
     * @param y              the center y-coordinate of the footprint
     * @param bottomZ        the bottom Z coordinate of the first layer
     * @param rad            the rotation angle of the footprint in radians
     * @param xside          the size of the footprint in the x direction
     * @param yside          the size of the footprint in the y direction
     * @param sphereDiameter the diameter of each ball
     * @param num            the index of the ball
     * @return the position of the num-th ball packed inside the rotated rectangular
     *         footprint
     */
    public static Translation3d packElWithRot (double x, double y, double bottomZ, double rad, double xside, double yside, double sphereDiameter, int num)
    {
        double eX = 0.092; // Error Adjustment
        double eY = 0.092; // Error Adjustment
        
        double cX = (xside - ((int) (xside / sphereDiameter) - 1) * sphereDiameter) / 2; // Centering Adjustment
        double cY = (yside - ((int) (yside / sphereDiameter) - 1) * sphereDiameter) / 2; // Centering Adjustment
        double x0 = x + Math.cos(Math.PI/4 + rad) * (sphereDiameter/2 + cX + eX); // Starting x of first fuel in robot
        double y0 = y + Math.sin(Math.PI/4 + rad) * (sphereDiameter/2 + cY + eY); // Starting y of first fuel in robot

        double z0 = bottomZ; // Starting z of first fuel

        int xn = (int) (xside / sphereDiameter) - 1; // num of fuel that can fit in x direction adjusting for bumper thickness
        int yn = (int) (yside / sphereDiameter) - 1; // num of fuel that can fit in y direction adjusting for bumper thickness

        int zN = num / (xn * yn); // layer number for current fuel

        double nx = num % xn; // number of fuel in x direction in current row
        double ny = (num % (xn * yn)) / xn; // number of fuel in y direction in current column

        double ix = Math.cos(rad) * sphereDiameter; // x increment for each fuel in row
        double iy = Math.sin(rad) * sphereDiameter; // y increment for each fuel in row

        double xc = x0 + nx * ix + ny * Math.cos(Math.PI/2 + rad) * sphereDiameter; // x coordinate of fuel based on number of fuel and increments
        double yc = y0 + nx * iy + ny * Math.sin(Math.PI/2 + rad) * sphereDiameter; // y coordinate of fuel based on number of fuel and increments
        
        return new Translation3d(xc + Math.sin(rad)*Constants.ROBOT_WIDTH/2, yc - Math.cos(rad)*Constants.ROBOT_HEIGHT/2, z0 + zN * sphereDiameter);
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
     * Rotates a rectangle's center position to the mirrored location on the other side of the field.
     * Applies field coordinate transforms to both X and Y to flip the position.
     */
    public static Rectangle2d rotate(Rectangle2d r)
    {
        return new Rectangle2d(new Pose2d(new Translation2d(Constants.Simulation.ROTATE_X.apply(r.getCenter().getX()), Constants.Simulation.ROTATE_Y.apply(r.getCenter().getY())), new Rotation2d()), r.getXWidth(), r.getYWidth());
    }

    /**
     * Turns a 2D translation to a 3D translation by adding a z-coordinate
     * 
     * @param translation2d the 2D translation
     * @param z             the z-coordinate
     * @return the 3D translation
     */
    public static Translation3d translation2dTo3d(Translation2d translation2d, double z)
    {
        return new Translation3d(translation2d.getX(), translation2d.getY(), z);
    }

    /**
     * Applies the alliance-based field transformation to a 3D translation
     * 
     * @param translation3d the translation to transform
     */
    public static Translation3d applyAlliance(Translation3d translation3d)
    {
        return (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) ? 
                translation3d : 
                translation2dTo3d(FlippingUtil.flipFieldPosition(translation3d.toTranslation2d()), translation3d.getZ());
    }

   /**
   * Calculates the distance in meters from the drivetrain to the scoring hub.
   * Flips the hub position based on alliance color to always target the correct goal.
   */
    public static double calculateShootDistance(CommandSwerveDrivetrain drivetrain)
    {
        Translation2d drivetrainPose = drivetrain.getState().Pose.getTranslation();
        
        Translation2d targetPose = (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) ? 
                    Constants.AlignConstants.HUB : 
                    FlippingUtil.flipFieldPosition(Constants.AlignConstants.HUB);
        return drivetrainPose.getDistance(targetPose);
    }
    
    /**
     * Calculates the shooting pitch angle based on the drivetrain pose and the hub target position
     * @param drivetrain    the drivetrain subsystem
     * @return  the calculated shooting pitch angle
     */
    public static Angle calculateShootPitch(CommandSwerveDrivetrain drivetrain)
    {
        Pair<LinearVelocity, Angle> value = interpolatingTreeMap.get(calculateShootDistance(drivetrain));
        if (value != null) 
        {
            Angle output = value.getSecond();
            if (output != null) 
            {
                return output;
            }
        }
        return Degrees.zero();
    }

    /**
     * Calculates the shooting pitch velocity based on the drivetrain pose and the hub target position
     * @param drivetrain    the drivetrain subsystem
     * @return  the calculated shooting pitch velocity
     */
    public static double calculateShootVelocity(CommandSwerveDrivetrain drivetrain)
    {
        Pair<LinearVelocity, Angle> value = interpolatingTreeMap.get(calculateShootDistance(drivetrain));
        if (value != null) 
        {
            LinearVelocity first = value.getFirst();
            if (first != null) 
            {
                double output = first.in(MetersPerSecond);
                return output;
            }
        }

        return 0.0;
    }
    
    /**
     * clamps a value between a minimum and maximum
     * 
     * @param value the value to clamp
     * @param min   the minimum
     * @param max   the maximum
     * @return the clamped value
     */
    public static double bound(double value, double min, double max)
    {
        if (value < min) return min;
        if (value > max) return max;
        return value;
    }

    /**
     * checks if the robot is on the left side of the field
     * 
     * @param drivetrain the drivetrain subsystem
     * @return true if the robot is on the left side; false otherwise
     */
    public static boolean onLeftSide(CommandSwerveDrivetrain drivetrain)
    {
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red)
        {
            
            return drivetrain.getState().Pose.getY() > Constants.Simulation.FIELD_HEIGHT.in(Meters) / 2.0;
        }
        else
        {
            return drivetrain.getState().Pose.getY() < Constants.Simulation.FIELD_HEIGHT.in(Meters) / 2.0;
        }
    }

}