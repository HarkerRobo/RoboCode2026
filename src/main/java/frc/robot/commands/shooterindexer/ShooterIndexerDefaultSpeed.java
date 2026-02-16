package frc.robot.commands.shooterindexer;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterIndexer;

public class ShooterIndexerDefaultSpeed extends Command
{
    public ShooterIndexerDefaultSpeed ()
    {
        addRequirements(ShooterIndexer.getInstance());
    }

    @Override
    public void initialize()
    {
        ShooterIndexer.getInstance().setVoltage(Volts.of(Constants.ShooterIndexer.DEFAULT_VOLTAGE));
    }

    @Override
    public void execute()
    {
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }

    @Override
    public void end(boolean interrupted)
    {
        ShooterIndexer.getInstance().setVoltage(Volts.of(0.0));
    }
    
}
