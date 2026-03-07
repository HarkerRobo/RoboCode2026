package frc.robot.commands.intake;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.intake.Intake;

public class EjectIntake extends Command 
{
    public EjectIntake () 
    {
        addRequirements(Intake.getInstance());
    }

    @Override
    public void initialize()
    {
        // Intake.getInstance().setVoltage(Volts.of(-2.5));
        Intake.getInstance().setVelocity(RotationsPerSecond.of(Constants.Intake.EJECT_VELOCITY));
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
        Intake.getInstance().setVoltage(Volts.zero());
        // Intake.getInstance().setVelocity(RotationsPerSecond.zero());
    }
}
