package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Indexer;

import static edu.wpi.first.units.Units.*;

public class IndexerFullSpeed extends Command
{
    
    public IndexerFullSpeed()
    {
        addRequirements(Indexer.getInstance());
    }

    @Override
    public void initialize() 
    {
        Indexer.getInstance().setMainVelocity(RotationsPerSecond.of(Constants.Indexer.MAIN_MAX_VELOCITY));
        Indexer.getInstance().setSideVelocity(RotationsPerSecond.of(Constants.Indexer.SIDE_MAX_VELOCITY));
    }

    @Override
    public boolean isFinished() 
    {
        return false;
    }
}
