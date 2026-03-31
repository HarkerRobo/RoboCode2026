package frc.robot.commands.shooter;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;
import frc.robot.util.Util;

public class ShooterTargetSpeedInPerpetuity extends Command
{
    DoubleSupplier targetSpeedSupplier;
    double targetSpeed;
    
    /**
     * Claims the Shooter subsystem and stores the supplied target speed source.
     * Allows retrieval of the desired flywheel surface velocity.
     */
    public ShooterTargetSpeedInPerpetuity(DoubleSupplier targetSpeedSupplier)
    {
        this.targetSpeedSupplier = targetSpeedSupplier;
        addRequirements(Shooter.getInstance());
    }

    /**
     * Claims the Shooter subsystem
     * Stores the supplied target speed source.
     */
    public ShooterTargetSpeedInPerpetuity(double leftTargetSpeed)
    {
        this(()->leftTargetSpeed);
    }
    
    /**
     * Reads the target speed and applies it as linear velocity.
     */
    @Override
    public void execute()
    {
        targetSpeed = targetSpeedSupplier.getAsDouble();
        Shooter.getInstance().setEffectiveVelocity(MetersPerSecond.of(targetSpeed));
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
