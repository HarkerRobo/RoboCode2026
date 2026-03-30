package frc.robot.commands.shooterindexer;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterIndexer;

public class ShooterIndexerStartDefaultSpeed extends Command
{
    public ShooterIndexerStartDefaultSpeed ()
    {
        addRequirements(ShooterIndexer.getInstance());
    }

    @Override
    public void initialize()
    {
        ShooterIndexer.getInstance().setVoltage(Volts.of(Constants.ShooterIndexer.DEFAULT_VOLTAGE));
    }

    @Override
    public boolean isFinished()
    {
        return true;
    }
}
