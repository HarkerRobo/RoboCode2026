package frc.robot.commands.shooter;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;
import frc.robot.util.Util;

public class ShooterTargetSpeed extends Command
{
    DoubleSupplier targetSpeedSupplier;
    double targetSpeed;

    /**
     * Claims the Shooter subsystem
     * Stores the supplied target speed source.
     */
    public ShooterTargetSpeed(DoubleSupplier targetSpeedSupplier)
    {
        this.targetSpeedSupplier = targetSpeedSupplier;
        addRequirements(Shooter.getInstance());
    }

    /**
     * Wraps target speed.
     */
    public ShooterTargetSpeed(double targetSpeed)
    {
        this(() -> targetSpeed);
    }

    /**
     * No initialization action is required.
     */
    @Override
    public void initialize()
    {
    }

    /**
     * Updates the shooter velocity using the bounded target speed.
     */
    @Override
    public void execute()
    {
        targetSpeed = Util.bound(targetSpeedSupplier.getAsDouble(), 0.0, Constants.Shooter.MAX_VELOCITY);
        Shooter.getInstance().setVelocity(RotationsPerSecond.of(targetSpeed));
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
