package frc.robot.commands.shooter;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

public class ShooterDefaultSpeed extends Command
{
    public ShooterDefaultSpeed()
    {
        addRequirements(Shooter.getInstance());
    }

    @Override
    public void initialize()
    {
        Shooter.getInstance().setVelocity(RotationsPerSecond.of(Constants.Shooter.DEFAULT_VELOCITY));
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
    }
}
