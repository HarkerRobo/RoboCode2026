package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Indexer;

import static edu.wpi.first.units.Units.*;

public class IndexerStartFullSpeed extends Command
{
    /**
     * Claims indexer subsystem.
     */
    public IndexerStartFullSpeed()
    {
        addRequirements(Indexer.getInstance());
    }

    /**
     * Sets main voltage to max.
     */
    @Override
    public void initialize() 
    {
        Indexer.getInstance().setMainVoltage(Volts.of(Constants.Indexer.MAIN_MAX_VOLTAGE));
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
