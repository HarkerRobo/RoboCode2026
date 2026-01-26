package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hopper;

public class ExtendHopper extends Command
{
    private double target; //rotations

    public ExtendHopper(double target)
    {
        this.target = target;

        addRequirements(Hopper.getInstance());
    }

    @Override
    public void initialize ()
    {
        Hopper.getInstance().setDesiredPosition(target);
        System.out.println("moving: " + target + "°");
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
        return "Extend Hopper (" + target + ")";
    }
}
