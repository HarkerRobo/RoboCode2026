package frc.robot.util;


import static edu.wpi.first.units.Units.Degrees;
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

    private static void addData(double distanceMeters, double velocityMetersPerSecond, double angleDegrees)
    {
        interpolatingTreeMap.put(distanceMeters, Pair.of(MetersPerSecond.of(velocityMetersPerSecond), Degrees.of(angleDegrees)));
    }

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

        // 2026-03-28 DATA POINTS

        double angleOffset = 0.0;
        double velocityOffset = 0.5;//0.4;
        addData(1.425, 7.4, 75.0 + angleOffset);
        addData(1.864, 7.7, 71.5 + angleOffset); // undershot with one motor running
        addData(2.062, 8.0, 71.5 + angleOffset);
        addData(2.320, 8.3, 71.5 + angleOffset); //2.x67 (auton place) 8.38, true angle: 70.6 (no offset)
        addData(2.614, 8.4 + velocityOffset - 0.1, 71.0 + angleOffset);
        addData(2.919, 8.5 + velocityOffset - 0.1, 70.8 + angleOffset);
        addData(3.574, 8.8 + velocityOffset, 70.3 + angleOffset);
        addData(3.952, 9.26 + velocityOffset, 70.1 + angleOffset);
        addData(4.312, 9.35 + velocityOffset, 69.9 + angleOffset); // should be tested with both motors running
        // addData(4.646, 9.7, 69.7 + angleOffset); // "
        // addData(5.697, 10.2, 64.9 + angleOffset); // "; farthest point
        
        addData(3.207, 8.35+ velocityOffset, 70.6 + angleOffset);
        addData(4.0, 9.25+ velocityOffset, 70.0 + angleOffset);
        addData(3.40, 9.0+ velocityOffset, 70.3 + angleOffset);
        addData(3.80, 9.2+ velocityOffset, 70.1 + angleOffset);

        addData(5.20, 9.5+ velocityOffset, 67.5 + angleOffset);
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
     * Rotates a rectangle's center
     * @param r the rectangle
     * @return  the new transformed rectangle
     */
    public static Rectangle2d rotate(Rectangle2d r)
    {
        return new Rectangle2d(new Pose2d(new Translation2d(Constants.Simulation.ROTATE_X.apply(r.getCenter().getX()), Constants.Simulation.ROTATE_Y.apply(r.getCenter().getY())), new Rotation2d()), r.getXWidth(), r.getYWidth());
    }

    public static Translation3d translation2dTo3d(Translation2d translation2d, double z)
    {
        return new Translation3d(translation2d.getX(), translation2d.getY(), z);
    }

    public static Translation3d applyAlliance(Translation3d translation3d)
    {
        return (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) ? 
                translation3d : 
                translation2dTo3d(FlippingUtil.flipFieldPosition(translation3d.toTranslation2d()), translation3d.getZ());
    }

    public static Translation3d getShootStartingPoint(CommandSwerveDrivetrain drivetrain)
    {
        Translation3d rawPose = translation2dTo3d(drivetrain.getState().Pose.getTranslation(), 0.0);
        return new Pose3d(rawPose, Rotation3d.kZero).transformBy(Constants.ROBOT_TO_HOOD).getTranslation();
    }

    public static Translation3d getShootEndingPoint()
    {
        return applyAlliance(Constants.HUB_TARGET_POSITION);
    }

    public static Translation3d getPassEndingPoint(CommandSwerveDrivetrain drivetrain)
    {
        return onLeftSide(drivetrain) ?
                applyAlliance(Constants.PASS_LEFT_TARGET_POSITION) : 
                applyAlliance(Constants.PASS_RIGHT_TARGET_POSITION);
    }

    public static double calculateShootDistance(CommandSwerveDrivetrain drivetrain)
    {
        Translation2d drivetrainPose = drivetrain.getState().Pose.getTranslation();
        
        Translation2d targetPose = (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) ? 
                    Constants.HUB_TARGET_POSITION.toTranslation2d() : 
                    FlippingUtil.flipFieldPosition(Constants.HUB_TARGET_POSITION.toTranslation2d());
        return drivetrainPose.getDistance(targetPose);
    }
    
    public static Angle calculatePitch(Translation3d position, Translation3d target, double shootVelocity)
    {
        double dx = target.getX() - position.getX();
        double dy = target.getY() - position.getY();
        double db = Math.sqrt(dx * dx + dy * dy);
        double dz = target.getZ() - position.getZ();
        double s = shootVelocity;
        double g = Constants.G;
        //System.out.println("\n\tdb: " + db + "\tdz: " + dz + "\ts: " + s);
        double idealAngle = Math.atan((Math.pow(s,2.0) + 
            Math.sqrt(Math.pow(s,4.0)
                - g * (g * Math.pow(db,2.0) + 2.0 * dz * Math.pow(s,2.0))))
            / (g * db));
        return Radians.of(idealAngle);
    }

    /**
     * Calculates the launch velocity from a position to the target
     * @param position  the start position
     * @param target    the target position
     * @return  the calculated launch velocity
     */
    public static double calculateVelocity(Translation3d position, Translation3d target)
    {
        double dx = target.getX() - position.getX();
        double dy = target.getY() - position.getY();
        double db = Math.sqrt(dx * dx + dy * dy);
        return Constants.DISTANCE_SHOOTVELO_RATIO * (db - Constants.Simulation.HUB_CONTENTS.getXWidth() / 2.0 - 6.0) + 10.0 - Constants.SPEED_OFFSET.in(MetersPerSecond);
    }

    /**
     * Calculates the shooting pitch angle based on the drivetrain pose and the hub target position
     * @param drivetrain    the drivetrain subsystem
     * @return  the calculated shooting pitch angle
     */
    public static Angle calculateShootPitch(CommandSwerveDrivetrain drivetrain)
    {
        if (Constants.INTERPOLATE_VALUES)
        {
            Pair<LinearVelocity, Angle> value = interpolatingTreeMap.get(calculateShootDistance(drivetrain));
            if (value != null)
            {
                Angle output = value.getSecond();
                if (output != null)
                {
                    System.out.printf("Calculated angle with interpolation: %f degrees\n", output.in(Degrees));
                    return output;
                }
            }
            // System.out.println("Interpolation failed - no value found. Reverting to math");
        }
        // Angle output = Util.calculatePitch(getShootStartingPoint(drivetrain), getShootEndingPoint(), calculateShootVelocity(drivetrain));
        // System.out.printf("Calculated angle: %f degrees\n", output.in(Degrees));
        return Degrees.zero();
    }

    /**
     * Calculates the shooting pitch velocity based on the drivetrain pose and the hub target position
     * @param drivetrain    the drivetrain subsystem
     * @return  the calculated shooting pitch velocity
     */
    public static double calculateShootVelocity(CommandSwerveDrivetrain drivetrain)
    {
        if (Constants.INTERPOLATE_VALUES)
        {
            Pair<LinearVelocity, Angle> value = interpolatingTreeMap.get(calculateShootDistance(drivetrain));
            if (value != null)
            {
                LinearVelocity first = value.getFirst();
                if (first != null)
                {
                    double output = first.in(MetersPerSecond);
                    System.out.printf("Calculated velocity with interpolation: %f\n", output);
                    return output;
                }
            }
            System.out.println("Interpolation failed - no value found. Reverting to math");
        }
        return 0.0;
        // double output = Util.calculateVelocity(getShootStartingPoint(drivetrain), getShootEndingPoint());
        // System.out.printf("Calculated velocity: %f\n", output);
        // return output;
    }
    
    public static Angle calculatePassPitch(CommandSwerveDrivetrain drivetrain)
    {
        return Degrees.of(65.0);
        //return Util.calculatePitch(getShootStartingPoint(drivetrain), getPassEndingPoint(drivetrain), calculatePassVelocity(drivetrain));
    }
    
    public static double calculatePassVelocity(CommandSwerveDrivetrain drivetrain)
    {
        return 12.7;
        //return Util.calculateVelocity(getShootStartingPoint(drivetrain), getPassEndingPoint(drivetrain));
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