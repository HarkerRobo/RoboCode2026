package frc.robot.commands.shooterindexer;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterIndexer;

public class ShooterIndexerStartFullSpeed extends Command
{
    public ShooterIndexerStartFullSpeed ()
    {
        addRequirements(ShooterIndexer.getInstance());
    }

    @Override
    public void initialize()
    {
        ShooterIndexer.getInstance().setVelocity(RotationsPerSecond.of(Constants.ShooterIndexer.INTAKE_VELOCITY));
    }

    @Override
    public boolean isFinished()
    {
        return true;
    }
}

