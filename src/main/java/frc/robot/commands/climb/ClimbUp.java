package frc.robot.commands.climb;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Climb;

public class ClimbUp extends Command
{
    /**
     * Claims the Climb subsystem.
     */
    public ClimbUp ()
    {
        addRequirements(Climb.getInstance());
    }

    /**
     * No initialization action is required.
     */
    @Override
    public void initialize ()
    {
        
    }

    /**
     * Drives the climb wheels upward at the configured voltage.
     */
    @Override
    public void execute ()
    {
        Climb.getInstance().setClimbWheelsVoltage((Constants.Climb.CLIMB_UP_VOLTAGE));
    }

    /**
     * This command never finishes on its own.
     */
    @Override
    public boolean isFinished ()
    {
        return false;
    }

    /**
     * Sets the climb wheel voltage to zero when the command ends.
     */
    @Override
    public void end (boolean interrupted)
    {
        Climb.getInstance().setClimbWheelsVoltage(Volts.of(0.0));
    }
}
