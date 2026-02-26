package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Indexer;

import static edu.wpi.first.units.Units.*;

public class IndexerFullSpeed extends Command
{
    /**
     * Claims the Indexer subsystem.
     */
    public IndexerFullSpeed()
    {
        addRequirements(Indexer.getInstance());
    }

    /**
     * Sets the indexer to its maximum rotational speed.
     */
    @Override
    public void initialize() 
    {
        Indexer.getInstance().setVelocity(RotationsPerSecond.of(Constants.Indexer.MAX_VELOCITY));
    }

    /**
     * This command never finishes on its own.
     */
    @Override
    public boolean isFinished() 
    {
        return false;
    }
}
