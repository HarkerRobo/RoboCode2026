package frc.robot.commands.hood;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.*;
import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Turret;

public class AimToAngle extends Command
{
    private double pitch; // degrees

    /**
     * 
     * @param pitch degrees
    */
    public AimToAngle (double pitch)
    {
        this.pitch = pitch;

        addRequirements(Hood.getInstance());
    }

    @Override
    public void initialize ()
    {
        Hood.getInstance().moveToPosition(Degrees.of(pitch));
        //System.out.println("Aiming: " + pitch + "°");
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
