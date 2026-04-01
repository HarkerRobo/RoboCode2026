package frc.robot.commands.shooter;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.function.DoubleSupplier;

import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;
import frc.robot.util.Util;

public class ShooterTargetSpeed extends Command
{
    DoubleSupplier targetSpeedSupplier;
    LinearVelocity targetSpeed;
    
    /**
     * Claims the Shooter subsystem and stores the supplied target speed source.
     * Allows retrieval of the desired flywheel surface velocity.
     */
    public ShooterTargetSpeed(DoubleSupplier targetSpeedSupplier)
    {
        this.targetSpeedSupplier = targetSpeedSupplier;
        addRequirements(Shooter.getInstance());
    }

    /**
     * Claims the Shooter subsystem
     * Stores the supplied target speed source.
     */
    public ShooterTargetSpeed(LinearVelocity targetSpeed)
    {
        this(()->targetSpeed.in(MetersPerSecond));
    }
    
    /**
     * Reads the target speed and applies it as linear velocity.
     */
    @Override
    public void initialize()
    {
        targetSpeed = MetersPerSecond.of(targetSpeedSupplier.getAsDouble());
        Shooter.getInstance().setEffectiveVelocity(targetSpeed);
    }

    /**
     * This command never finishes on its own.
     */
    @Override
    public boolean isFinished()
    {
        return true;
    }

    /**
     * No cleanup is required.
     */
    @Override
    public void end(boolean interrupted)
    {
    }
}
