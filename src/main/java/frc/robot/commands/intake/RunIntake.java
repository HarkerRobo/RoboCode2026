package frc.robot.commands.intake;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.intake.Intake;

public class RunIntake extends Command 
{
    public RunIntake () 
    {
        addRequirements(Intake.getInstance());
    }

    @Override
    public void initialize()
    {
        Intake.getInstance().setMainVoltage(Volts.of(Constants.Intake.INTAKE_VOLTAGE));
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
        Intake.getInstance().setMainVoltage(Volts.of(Constants.Intake.DEFAULT_INTAKE_VOLTAGE));
    }

    @Override
    public String getName()
    {
        return "RunIntake";
    }
}
