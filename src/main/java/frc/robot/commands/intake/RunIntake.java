package frc.robot.commands.intake;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.intake.Intake;

public class RunIntake extends Command 
{
    /**
     * Claims the Intake subsystem.
     */
    public RunIntake() 
    {
        addRequirements(Intake.getInstance());
    }

    /**
     * Applies the configured intake voltage.
     */
    @Override
    public void initialize()
    {
        // Intake.getInstance().setVoltage(Volts.of(5.5));
        Intake.getInstance().setVelocity(RotationsPerSecond.of(Constants.Intake.INTAKE_VELOCITY));
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
     * Restores the intake to its default voltage when the command ends.
     */
    @Override
    public void end(boolean interrupted) 
    {
        Intake.getInstance().setVoltage(Volts.zero());
        // Intake.getInstance().setVelocity(RotationsPerSecond.zero());
    }
}
