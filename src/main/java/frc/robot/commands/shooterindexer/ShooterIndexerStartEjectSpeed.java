package frc.robot.commands.shooterindexer;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterIndexer;

public class ShooterIndexerStartEjectSpeed extends Command
{
    public ShooterIndexerStartEjectSpeed ()
    {
        addRequirements(ShooterIndexer.getInstance());
    }

    @Override
    public void initialize()
    {
        ShooterIndexer.getInstance().setVelocity(RotationsPerSecond.of(Constants.ShooterIndexer.EJECT_VELOCITY));
    }

    @Override
    public boolean isFinished()
    {
        return true;
    }
}

