package frc.robot.commands.intake;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

/**
 * is still used but applies a zero voltage
 */
public class DefaultIntake extends Command 
{
    /**
     * Claims the Intake subsystem.
     */
    public DefaultIntake() 
    {
        addRequirements(Intake.getInstance());
    }

    /**
     * Applies the default intake voltage.
     */
    @Override
    public void initialize()
    {
        Intake.getInstance().setVoltage(Volts.zero());
        // Intake.getInstance().setVelocity(RotationsPerSecond.zero());
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
     * Sets the intake voltage to zero when the command ends.
     */
    @Override
    public void end(boolean interrupted) 
    {
        Intake.getInstance().setVoltage(Volts.zero());
        // Intake.getInstance().setVelocity(RotationsPerSecond.zero());
    }
}
