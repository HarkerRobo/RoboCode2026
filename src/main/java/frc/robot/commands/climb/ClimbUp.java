package frc.robot.commands.climb;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Climb;

public class ClimbUp extends Command
{
    public ClimbUp ()
    {
        addRequirements(Climb.getInstance());
    }

    @Override
    public void initialize ()
    {
        
    }

    @Override
    public void execute ()
    {
        Climb.getInstance().setClimbWheelsVoltage((Constants.Climb.CLIMB_UP_VOLTAGE));
    }

    @Override
    public boolean isFinished ()
    {
        return false;
    }

    @Override
    public void end (boolean interrupted)
    {
        Climb.getInstance().setClimbWheelsVoltage(Volts.of(0.0));
    }
}
