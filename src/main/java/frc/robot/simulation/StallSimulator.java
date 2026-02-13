package frc.robot.simulation;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

public class StallSimulator
{
    public static List<StallSimulator> instances = new ArrayList<>(10);

    public static void update()
    {
        for (StallSimulator s : instances) s.periodic();
    }

    public Supplier<Double> position;
    public double lastLastPosition = -1.0;
    public double lastPosition = -1.0;
    public double currentPosition = -1.0;

    public StallSimulator(Supplier<Double> position)
    {
        this.position = position;
        instances.add(this);
    }

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

    public boolean get()
    {
        if (lastLastPosition == -1.0 || lastPosition == -1.0 || currentPosition == -1.0) return false;

        return Math.abs(lastLastPosition - lastPosition) < 0.01 && Math.abs(lastPosition - currentPosition) < 0.01;
    }
}
