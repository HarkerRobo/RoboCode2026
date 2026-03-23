package frc.robot.commands.hood;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Hood;

public class HoodManualUp extends Command
{
    /**
     * Claims the Hood subsystem.
     */
    public HoodManualUp()
    {
        addRequirements(Hood.getInstance());
    }
    
    /**
     * Drives the hood upward at the configured manual voltage.
     */
    @Override
    public void initialize()
    {
        Hood.getInstance().setVoltage(Volts.of(Constants.Hood.MANUAL_UP_VOLTAGE));
    }

    /**
     * No repeated action is required.
     */
    @Override
    public void execute()
    {
    }

    /**
     * Finishes when the hood stalls against its mechanical stop.
     */
    @Override
    public boolean isFinished()
    {
        return Hood.getInstance().isStalling();
    }

    /**
     * Holds the hood at its current position when the command ends.
     */
    @Override
    public void end(boolean interrupted)
    {
        Hood.getInstance().moveToPosition(Hood.getInstance().getPosition());
    }
}
