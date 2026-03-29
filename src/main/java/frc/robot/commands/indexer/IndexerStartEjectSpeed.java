package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Indexer;

import static edu.wpi.first.units.Units.*;

public class IndexerStartEjectSpeed extends Command
{
    
    public IndexerStartEjectSpeed()
    {
        addRequirements(Indexer.getInstance());
    }

    @Override
    public void initialize() 
    {
        Indexer.getInstance().setMainVoltage(Volts.of(Constants.Indexer.MAIN_EJECT_VOLTAGE));
    }

    @Override
    public boolean isFinished() 
    {
        return true;
    }
}
