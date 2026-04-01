package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Indexer;

import static edu.wpi.first.units.Units.*;

public class IndexerStartEjectSpeed extends Command
{
    /**
     * Claims the Indexer subsystem and starts it at the eject voltage.
     */
    public IndexerStartEjectSpeed()
    {
        addRequirements(Indexer.getInstance());
    }

    /**
     * Sets the main indexer motor to its eject operating voltage.
     */
    @Override
    public void initialize() 
    {
        Indexer.getInstance().setMainVoltage(Constants.Indexer.MAIN_EJECT_VOLTAGE);
    }

    /**
     * Command finishes immediately.
     */
    @Override
    public boolean isFinished() 
    {
        return true;
    }
}
