package frc.robot.commands.shooter;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

public class ShooterDefaultSpeed extends Command
{
    /**
     * Claims the Shooter subsystem.
     */
    public ShooterDefaultSpeed()
    {
        addRequirements(Shooter.getInstance());
    }

    /**
     * Sets the shooter to its configured default velocity.
     */
    @Override
    public void initialize()
    {
        Shooter.getInstance().setEffectiveVelocity(MetersPerSecond.of(Constants.Shooter.DEFAULT_VELOCITY));
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
     * No cleanup is required.
     */
    @Override
    public void end(boolean interrupted)
    {
    }
}
