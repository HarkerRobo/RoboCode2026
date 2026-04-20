package frc.robot.commands.shooterindexer;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterIndexer;

public class ShooterIndexerStartFullSpeed extends Command
{
    /**
     * Claims subsys.
     */
    public ShooterIndexerStartFullSpeed ()
    {
        addRequirements(ShooterIndexer.getInstance());
    }

    /**
     * Sets the shooter‑indexer motor to its full‑speed intake velocity.
     */
    @Override
    public void initialize()
    {
        ShooterIndexer.getInstance().setVelocity(Constants.ShooterIndexer.INTAKE_VELOCITY);
    }

    /**
     * Completes immediately.
     */
    @Override
    public boolean isFinished()
    {
        return true;
    }
}

