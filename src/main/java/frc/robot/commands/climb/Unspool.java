package frc.robot.commands.climb;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Climb;

public class Unspool extends Command
{
    /**
     * Claims the Climb subsystem.
     */
    public Unspool()
    {
        addRequirements(Climb.getInstance());
    }

    /**
     * Applies the configured unspooling voltage.
     */
    @Override
    public void initialize()
    {
        Climb.getInstance().setSpoolingVoltage(Constants.Climb.UNSPOOL_VOLTAGE);
    }

    /**
     * No repeated action is required.
     */
    @Override
    public void execute()
    {
    }

    /**
     * This command never finishes on its own.
     */
    @Override
    public boolean isFinished()
    {
        return false;
    }

    /**
     * Sets the spooling voltage to zero when the command ends.
     */
    @Override
    public void end(boolean interrupted)
    {
        Climb.getInstance().setSpoolingVoltage((Volts.of(0.0)));
    }
}
