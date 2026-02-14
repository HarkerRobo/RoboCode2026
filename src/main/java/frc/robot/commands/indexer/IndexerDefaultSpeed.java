package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Indexer;

import static edu.wpi.first.units.Units.*;

public class IndexerDefaultSpeed extends Command{
    
    public IndexerDefaultSpeed() 
    {
        addRequirements(Indexer.getInstance());
    }

    @Override
    public void initialize() 
    {
        Indexer.getInstance().setVelocity(RotationsPerSecond.of(Constants.Indexer.DEFAULT_VELOCITY));
    }

    @Override
    public boolean isFinished() 
    {
        return false;
    }
}
