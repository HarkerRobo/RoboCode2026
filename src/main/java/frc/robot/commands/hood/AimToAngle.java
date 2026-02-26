package frc.robot.commands.hood;

import static edu.wpi.first.units.Units.*;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Hood;
import frc.robot.util.Util;

public class AimToAngle extends Command
{
    private DoubleSupplier pitcher;
    private double pitch = 0.0; // degrees

    /**
     * Creates a command that aims the hood to a supplied angle in degrees.
     * Stores the supplier and claims the Hood subsystem.
     * @param pitch degrees
     */
    public AimToAngle(DoubleSupplier pitcher)
    {
        this.pitcher = pitcher;
        addRequirements(Hood.getInstance());
    }

    /**
     * Creates a command that aims the hood to a fixed angle in degrees.
     */
    public AimToAngle(double pitch)
    {
        this(() -> pitch);
    }

    /**
     * Reads the target angle, clamps it to hood limits, and commands the hood.
     * Prints a warning if the requested angle was out of bounds.
     */
    @Override
    public void initialize()
    {
        pitch = pitcher.getAsDouble();
        double boundedPitch = Util.bound(pitch, Constants.Hood.MIN_ANGLE, Constants.Hood.MAX_ANGLE);
        if (boundedPitch != pitch)
            System.out.println("Bounding out-of-bounds pitch " + pitch + " to " + boundedPitch);
        this.pitch = boundedPitch;

        Hood.getInstance().moveToPosition(Degrees.of(pitch));
        System.out.println("Aiming: " + pitch + "°");
    }

    /**
     * No repeated action required during execution.
     * Hood motion is handled internally by the subsystem.
     */
    @Override
    public void execute()
    {
    }

    /**
     * Finishes when the hood reports that it is within tolerance of the target.
     * Uses the subsystem’s readyToShoot() check.
     */
    @Override
    public boolean isFinished()
    {
        return Hood.getInstance().readyToShoot();
    }

    /**
     * No cleanup when the command ends.
     * Method included for Command API completeness.
     */
    @Override
    public void end(boolean interrupted)
    {
    }

    /**
     * Returns a readable name including the target angle.
     * Debugging and command tracing.
     */
    @Override
    public String getName()
    {
        return "AimToAngle (" + pitch + "°)";
    }
}
