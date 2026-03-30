package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Indexer;

import static edu.wpi.first.units.Units.*;

public class IndexerStartDefaultSpeed extends Command
{
    /**
     * Claims the Indexer subsystem😭🙏.
     */
    public IndexerStartDefaultSpeed() 
    {
        addRequirements(Indexer.getInstance());
    }

    
    /**
     * Sets the main indexer motor to its default operating voltage.
     */
    @Override
    public void initialize() 
    {
        Indexer.getInstance().setMainVoltage(Volts.of(Constants.Indexer.MAIN_DEFAULT_VOLTAGE));
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
