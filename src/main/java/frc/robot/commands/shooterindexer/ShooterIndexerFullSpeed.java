package frc.robot.commands.shooterindexer;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterIndexer;

public class ShooterIndexerFullSpeed extends Command
{
    /**
     * Claims the ShooterIndexer subsystem.
     */
    public ShooterIndexerFullSpeed()
    {
        addRequirements(ShooterIndexer.getInstance());
    }

    /**
     * Applies the configured shooter‑indexer voltage.
     */
    @Override
    public void initialize()
    {
        ShooterIndexer.getInstance().setVoltage(Volts.of(Constants.ShooterIndexer.INTAKE_VOLTAGE));
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
     * Sets the shooter‑indexer voltage to zero when the command ends.
     */
    @Override
    public void end(boolean interrupted)
    {
        ShooterIndexer.getInstance().setVoltage(Volts.of(0.0));
    }

}
