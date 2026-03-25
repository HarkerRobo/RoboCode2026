package frc.robot.commands.shooter;

import static edu.wpi.first.units.Units.MetersPerSecond;

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

    public ShooterTargetSpeed(double leftTargetSpeed)
    {
        this(()->leftTargetSpeed);
    }
    
    @Override
    public void initialize()
    {
        targetSpeed = targetSpeedSupplier.getAsDouble();
        Shooter.getInstance().setEffectiveVelocity(MetersPerSecond.of(targetSpeed));
    }

    @Override
    public boolean isFinished()
    {
        return true;
    }

    @Override
    public void end(boolean interrupted)
    {
    }
}
