package frc.robot.commands.intake;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.intake.Intake;

/**
 * Not currently used - will be used if we want to apply
 * a nonzero voltage while not intaking or want a more advanced
 * control than a simple toggle.
 */
public class DefaultIntake extends Command 
{
    public DefaultIntake () 
    {
        addRequirements(Intake.getInstance());
    }

    @Override
    public void initialize()
    {
        Intake.getInstance().setMainVoltage(Volts.of(Constants.Intake.DEFAULT_INTAKE_VOLTAGE));
    }

    @Override
    public void execute () 
    {
    }

    @Override
    public boolean isFinished () 
    {
        return false;
    }

    @Override
    public void end (boolean interrupted) 
    {
        Intake.getInstance().setMainVoltage(Volts.of(0.0));
    }
}
