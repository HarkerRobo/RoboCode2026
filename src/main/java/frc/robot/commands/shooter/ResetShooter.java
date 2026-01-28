package frc.robot.commands.shooter;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

public class ResetShooter extends Command
{
    public ResetShooter()
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
        return Math.abs(Shooter.getInstance().getVelocity().in(RotationsPerSecond) - Constants.Shooter.DEFAULT_VELOCITY) < 0.1;
    }

    @Override
    public void end(boolean interrupted)
    {
    }

    @Override
    public String getName()
    {
        return "ResetShooter";
    }
}
