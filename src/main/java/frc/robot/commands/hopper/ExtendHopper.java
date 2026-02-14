package frc.robot.commands.hopper;

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
        Hopper.getInstance().setVoltage(Volts.of(Constants.Hopper.FORWARD_VOLTAGE));
    }

    @Override
    public void execute ()
    {

    }

    @Override
    public boolean isFinished ()
    {
        return Hopper.getInstance().isStalling();
    }

    @Override
    public void end (boolean interrupted)
    {
        
    }
}
