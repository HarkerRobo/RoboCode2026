package frc.robot.commands.hopper;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Hopper;

public class RetractHopper extends Command
{
    public RetractHopper()
    {
        addRequirements(Hopper.getInstance());
    }

    @Override
    public void initialize ()
    {
        Hopper.getInstance().setDesiredPosition(Constants.Hopper.MIN_POSITION);
        // System.out.println("moving: " + Constants.Hopper.MIN_POSITION + " rotations");
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
        return "Retract Hopper (" + Constants.Hopper.MIN_POSITION + ")";
    }
}
