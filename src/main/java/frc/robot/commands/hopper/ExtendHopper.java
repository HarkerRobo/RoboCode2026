package frc.robot.commands.hopper;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.*;
import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Hopper;

public class ExtendHopper extends Command
{
    public ExtendHopper()
    {
        addRequirements(Hopper.getInstance());
    }

    @Override
    public void initialize ()
    {
        Hopper.getInstance().setDesiredPosition(Rotations.of(Constants.Hopper.EXTEND_POSITION));
        // System.out.println("moving: " + Constants.Hopper.EXTEND_POSITION + " rotations");
    }

    @Override
    public void execute ()
    {

    }

    @Override
    public boolean isFinished ()
    {
        return Hopper.getInstance().isStalling() || Hopper.getInstance().atPosition();
    }

    @Override
    public void end (boolean interrupted)
    {
        
    }

    @Override
    public String getName()
    {
        return "Extend Hopper (" + Constants.Hopper.EXTEND_POSITION + ")";
    }
}
