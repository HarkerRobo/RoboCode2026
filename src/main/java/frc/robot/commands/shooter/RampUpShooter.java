package frc.robot.commands.shooter;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

public class RampUpShooter extends Command
{
    public RampUpShooter()
    {
        addRequirements(Shooter.getInstance());
    }

    @Override
    public void initialize()
    {
        Shooter.getInstance().setVelocity(RotationsPerSecond.of(Constants.Shooter.SHOOT_VELOCITY));
    }

    @Override
    public void execute()
    {
    }

    @Override
    public boolean isFinished()
    {
        return Shooter.getInstance().getLeftVelocity().in(RotationsPerSecond) >= Constants.Shooter.SHOOT_VELOCITY;
    }

    @Override
    public void end(boolean interrupted)
    {
    }

    @Override
    public String getName()
    {
        return "RampUpShooter";
    }
}
