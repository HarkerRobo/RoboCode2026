package frc.robot.commands.intake;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.*;
import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.intake.Intake;

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

    @Override
    public String getName()
    {
        return "DefaultIntake";
    }
}
