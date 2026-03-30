package frc.robot.simulation;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
/**
 * Stall detector for simulation that checks for stall behavior
 * Reports stalled when the position is not effectively changing over multiple consecutive samples
 */
public class StallSimulator
{
    public static List<StallSimulator> instances = new ArrayList<>(10);
    /**
     * Updates all registered instances
     * Called once per simulation loop
     */
    public static void update()
    {
        for (StallSimulator s : instances) s.periodic();
    }

    public Supplier<Double> position;
    public double lastLastPosition = -1.0;
    public double lastPosition = -1.0;
    public double currentPosition = -1.0;
    /**
     * Creates a stall detector that samples from the provided position supplier
     * @param position  the position supplier
     */
    public StallSimulator(Supplier<Double> position)
    {
        this.position = position;
        instances.add(this);
    }
    /**
     * Updates the internal sample history
     * Sets the current position
     */
    public void periodic()
    {
        if (lastPosition != -1.0)
        {
            lastLastPosition = lastPosition;
        }

        if (currentPosition != -1.0)
        {
            lastPosition = currentPosition;
        }

        currentPosition = position.get();
    }
    /**
     * Returns whether the sampled position suggests a stall
     * @return  True if the sampled position suggests a stall; false otherwise
     */
    public boolean get()
    {
        if (lastLastPosition == -1.0 || lastPosition == -1.0 || currentPosition == -1.0) return false;

        return Math.abs(lastLastPosition - lastPosition) < 0.01 && Math.abs(lastPosition - currentPosition) < 0.01;
    }
}
