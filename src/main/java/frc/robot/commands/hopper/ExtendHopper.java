package frc.robot.commands.hopper;

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
        Hopper.getInstance().setDesiredPosition(Constants.Hopper.MAX_POSITION);
        // System.out.println("moving: " + Constants.Hopper.MAX_POSITION + " rotations");
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
        return "Extend Hopper (" + Constants.Hopper.MAX_POSITION + ")";
    }
}
