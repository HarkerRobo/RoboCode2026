package frc.robot.simulation;

/**
 * FRC 868 Code
 */
public class BallConstants 
{
    /**
     * Unit: Kilograms
     */
    public final double mass;
    /**
     * Unit: Meters
     */
    public final double radius;
    /**
     * Unit: Square meters
     */
    public final double area;

    /**
     * Unit: Kilogram per cubic meter
     */
    public final double rho;
    /**
     * Drag coefficient, unitless
     */
    public final double cd;
    /**
     * Lift coefficient gain, unitless
     */
    public final double clGain;
    /**
     * Maximum lift coefficient bound, unitless
     */
    public final double clMax;

    /**
     * Units: Meters per second squared
     */
    public final double gravity;
    /**
     * Unit: Seconds
     */
    public final double spinDecayTau;

    /**
     * Constructs ball constants
     * Including mass, radius, air density, drag coefficient, lift coefficient,
     * maximum lift coefficient, gravitational acceleration, and spin decay time
     */
    public BallConstants(
            double mass,
            double radius,
            double rho,
            double cd,
            double clGain,
            double clMax,
            double gravity,
            double spinDecayTau) 
    {

        this.mass = mass;
        this.radius = radius;
        this.area = Math.PI * radius * radius;

        this.rho = rho;
        this.cd = cd;
        this.clGain = clGain;
        this.clMax = clMax;

        this.gravity = gravity;
        this.spinDecayTau = spinDecayTau;
    }
}
