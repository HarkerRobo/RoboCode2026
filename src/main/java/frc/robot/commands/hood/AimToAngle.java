package frc.robot.commands.hood;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.*;
import static edu.wpi.first.units.Units.*;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Turret;
import frc.robot.util.Util;

public class AimToAngle extends Command
{
    private DoubleSupplier pitcher;
    private double pitch = 0.0; // degrees

    /**
     * 
     * @param pitch degrees
    */
    public AimToAngle (DoubleSupplier pitcher)
    {
        this.pitcher = pitcher;
        addRequirements(Hood.getInstance());
    }

    public AimToAngle (double pitch)
    {
        this(()->pitch);
    }

    @Override
    public void initialize ()
    {
        pitch = pitcher.getAsDouble();
        double boundedPitch = Util.bound(pitch, Constants.Hood.MIN_ANGLE, Constants.Hood.MAX_ANGLE);
        if (boundedPitch != pitch) System.out.println("Bounding out-of-bounds pitch " + pitch + "to " + boundedPitch);
        this.pitch = boundedPitch;

        Hood.getInstance().moveToPosition(Degrees.of(pitch));
        System.out.println("Aiming: " + pitch + "°");
    }

    @Override
    public void execute ()
    {
    }

    @Override
    public boolean isFinished ()
    {
        return Hood.getInstance().readyToShoot();
    }

    @Override
    public void end (boolean interrupted)
    {
    }

    @Override
    public String getName()
    {
        return "AimToAngle (" + pitch + "°)";
    }
}
