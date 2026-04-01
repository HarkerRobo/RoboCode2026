package frc.robot.simulation;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Constants;

/**
 * Modified FRC 868 Code
 */
public final class BallPhysics {
    
    public record ShotSolution(
            double launchPitchRad,
            double launchSpeed,
            double flightTimeSeconds) {
    }

    
    private BallPhysics() {
    }

    /**
     * Computes the gravitational force acting on the ball.
     * Returns a downward vector scaled by mass and gravity.
     */
    private static Translation3d gravityForce(BallConstants c) {
        return new Translation3d(0, 0, -c.mass * c.gravity);
    }

    /**
     * Computes aerodynamic drag based on velocity.
     * Returns a force opposite the direction of travel.
     */
    private static Translation3d dragForce(
            Translation3d v, BallConstants c) {

        double speed = v.getNorm();
        if (speed < 1e-6)
            return new Translation3d();

        double scale = -0.5 * c.rho * c.cd * c.area * speed;
        return v.times(scale);
    }

    /**
     * Computes the Magnus lift force from ball spin.
     * Returns a vector perpendicular to both spin and velocity.
     */
    private static Translation3d magnusForce(
            Translation3d v, Translation3d omega, BallConstants c) {

        double speed = v.getNorm();
        double wMag = omega.getNorm();
        if (speed < 1e-6 || wMag < 1e-6)
            return new Translation3d();

        double spinRatio = wMag * c.radius / speed;
        double cl = Math.min(c.clGain * spinRatio, c.clMax);

        Translation3d vHat = v.div(speed);
        Translation3d wHat = omega.div(wMag);

        Translation3d direction = new Translation3d(Vector.cross(wHat.toVector(), vHat.toVector()));
        double magnitude = 0.5 * c.rho * cl * c.area * speed * speed;

        return direction.times(magnitude);
    }

    /**
     * Integrates angular velocity into a new rotation.
     * Applies a small-angle approximation for incremental updates.
     */
    private static Rotation3d integrateRotation(
            Rotation3d current,
            Translation3d omega,
            double dt) {

        Rotation3d delta = new Rotation3d(
                omega.getX() * dt,
                omega.getY() * dt,
                omega.getZ() * dt);

        return current.plus(delta);
    }

    /**
     * Advances the ball state by one simulation step.
     * Updates position, velocity, spin, and rotation using physics forces.
     */
    public static void step(
            BallState s, BallConstants c, double dt) {

        Translation3d force = gravityForce(c)
                .plus(dragForce(s.velocity, c))
                .plus(magnusForce(s.velocity, s.omega, c));

        Translation3d accel = force.div(c.mass);

        double decay = Math.exp(-dt / c.spinDecayTau);
        s.omega = s.omega.times(decay);

        s.velocity = s.velocity.plus(accel.times(dt));
        
        if (s.pose.getZ() > 0.5 * Constants.Simulation.FUEL_DIAMETER.in(Meters) || s.velocity.getZ() > 0)
        {
            s.pose = new Pose3d(s.pose.getTranslation().plus(s.velocity.times(dt)),
                integrateRotation(s.pose.getRotation(), s.omega, dt));
        }
        else
        {
            s.pose = new Pose3d(s.pose.getX(), s.pose.getY(), 0.5 * Constants.Simulation.FUEL_DIAMETER.in(Meters), s.pose.getRotation());
        }
    }

    /**
     * Solves for a ballistic shot given a required incoming angle.
     * Computes launch pitch, launch speed, and flight time.
     */
    public static ShotSolution solveBallisticWithIncomingAngle(
            Pose3d shooterPose,
            Pose3d targetPose,
            double incomingPitchRad) {

        Translation3d s = shooterPose.getTranslation();
        Translation3d t = targetPose.getTranslation();

        double dx = t.getX() - s.getX();
        double dy = t.getY() - s.getY();
        double dz = t.getZ() - s.getZ();

        double d = Math.hypot(dx, dy);
        if (d < 1e-9) {
            throw new IllegalArgumentException("Horizontal distance too small");
        }

        double tanThetaT = Math.tan(incomingPitchRad);

        double rhs = dz - d * tanThetaT;
        if (rhs <= 0) {
            throw new IllegalArgumentException(
                    "No physical solution: dz - d*tan(thetaT) must be > 0");
        }

        double T = Math.sqrt(2.0 * rhs / Constants.G.in(MetersPerSecondPerSecond));

        double vHoriz = d / T;
        double vZ0 = vHoriz * tanThetaT + Constants.G.in(MetersPerSecondPerSecond) * T;

        double launchSpeed = Math.hypot(vHoriz, vZ0);
        double launchPitch = Math.atan2(vZ0, vHoriz);

        return new ShotSolution(launchPitch, launchSpeed, T);
    }

    /**
     * Solves for a ballistic shot given a fixed launch speed.
     * Computes the required pitch angle and flight time.
     */
    public static ShotSolution solveBallisticWithSpeed(
            Pose3d shooterPose,
            Pose3d targetPose,
            double launchSpeed) {

        Translation3d s = shooterPose.getTranslation();
        Translation3d t = targetPose.getTranslation();

        double dx = t.getX() - s.getX();
        double dy = t.getY() - s.getY();
        double dz = t.getZ() - s.getZ();

        double d = Math.hypot(dx, dy);
        if (d < 1e-9) {
            throw new IllegalArgumentException("Horizontal distance too small");
        }

        double v2 = launchSpeed * launchSpeed;
        double g = Constants.G.in(MetersPerSecondPerSecond);

        double discriminant = v2 * v2 - g * (g * d * d + 2.0 * dz * v2);
        if (discriminant < 0) {
            return new ShotSolution(0, 0, 0);
        }

        // LOW-ARC solution (use +Math.sqrt(...) for high arc)
        double tanTheta = (v2 + Math.sqrt(discriminant)) / (g * d);

        double launchPitch = Math.atan(tanTheta);

        double vHoriz = launchSpeed * Math.cos(launchPitch);
        double time = d / vHoriz;

        return new ShotSolution(launchPitch, launchSpeed, time);
    }

    /**
     * Computes the minimum possible launch speed for any valid arc.
     * Uses a closed-form solution based on gravity and geometry.
     */
    public static double minSpeedForAnyArc(
            Pose3d shooterPose,
            Pose3d targetPose) {

        Translation3d s = shooterPose.getTranslation();
        Translation3d t = targetPose.getTranslation();

        double dx = t.getX() - s.getX();
        double dy = t.getY() - s.getY();
        double dz = t.getZ() - s.getZ();

        double d = Math.hypot(dx, dy);

        return Math.sqrt(
            Constants.G.in(MetersPerSecondPerSecond) * (Math.hypot(d, dz) + dz));
    }
}