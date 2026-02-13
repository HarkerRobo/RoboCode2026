package frc.robot.commands.shooter;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

public class RampUpShooter extends Command
{
    DoubleSupplier targetSpeedSupplier;
    double targetSpeed;
    public RampUpShooter(DoubleSupplier targetSpeedSupplier)
    {
        this.targetSpeedSupplier = targetSpeedSupplier;
        addRequirements(Shooter.getInstance());
    }

    @Override
    public void initialize()
    {
        targetSpeed = targetSpeedSupplier.getAsDouble();
        Shooter.getInstance().setVelocity(RotationsPerSecond.of(targetSpeed));
    }

    @Override
    public void execute()
    {
    }

    @Override
    public boolean isFinished()
    {
        return Shooter.getInstance().getLeftVelocity().in(RotationsPerSecond) >= targetSpeed;
    }

    @Override
    public void end(boolean interrupted)
    {
    }
}
