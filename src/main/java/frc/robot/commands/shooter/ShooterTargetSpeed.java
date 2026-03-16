package frc.robot.commands.shooter;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;
import frc.robot.util.Util;

public class ShooterTargetSpeed extends Command
{
    DoubleSupplier leftTargetSpeedSupplier;
    double leftTargetSpeed;
    
    DoubleSupplier rightTargetSpeedSupplier;
    double rightTargetSpeed;
    
    /**
     * Claims the Shooter subsystem
     * Stores the supplied target speed source.
     */
    public ShooterTargetSpeed(DoubleSupplier leftTargetSpeedSupplier, DoubleSupplier rightTargetSpeedSupplier)
    {
        this.leftTargetSpeedSupplier = leftTargetSpeedSupplier;
        this.rightTargetSpeedSupplier = rightTargetSpeedSupplier;
        addRequirements(Shooter.getInstance());
    }

    public ShooterTargetSpeed(double leftTargetSpeed, double rightTargetSpeed)
    {
        this(()->leftTargetSpeed, ()->rightTargetSpeed);
    }
    
    public ShooterTargetSpeed(DoubleSupplier targetSpeedSupplier)
    {
        this(targetSpeedSupplier, targetSpeedSupplier);
    }

    public ShooterTargetSpeed(double targetSpeed)
    {
        this(()->targetSpeed);
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
        leftTargetSpeed = leftTargetSpeedSupplier.getAsDouble();
        Shooter.getInstance().setLeftEffectiveVelocity(MetersPerSecond.of(leftTargetSpeed));
        rightTargetSpeed = rightTargetSpeedSupplier.getAsDouble();
        Shooter.getInstance().setRightEffectiveVelocity(MetersPerSecond.of(rightTargetSpeed));
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
