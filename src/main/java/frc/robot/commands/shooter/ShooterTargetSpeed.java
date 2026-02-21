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
    public ShooterTargetSpeed(DoubleSupplier targetSpeedSupplier)
    {
        this.targetSpeedSupplier = targetSpeedSupplier;
        addRequirements(Shooter.getInstance());
    }

    public ShooterTargetSpeed(double targetSpeed)
    {
        this(()->targetSpeed);
    }

    @Override
    public void initialize()
    {
    }

    @Override
    public void execute()
    {
        targetSpeed = Util.bound(targetSpeedSupplier.getAsDouble(), 0.0, Constants.Shooter.MAX_VELOCITY);
        Shooter.getInstance().setVelocity(RotationsPerSecond.of(targetSpeed));
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
