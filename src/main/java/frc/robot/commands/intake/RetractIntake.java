package frc.robot.commands.intake;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class RetractIntake extends Command
{
    public RetractIntake()
    {
        addRequirements(Intake.getInstance());
    }

    @Override
    public void initialize()
    {
        Intake.getInstance().setExtensionVoltage(Volts.of(Constants.Intake.EXTENSION_RETRACTING_VOLTAGE));
    }

    @Override
    public void execute()
    {
    }

    @Override
    public boolean isFinished()
    {
        return Intake.getInstance().extensionIsStalling();
    }

    @Override
    public void end(boolean interrupted)
    {
        Intake.getInstance().setExtensionVoltage(Volts.of(0.0));
    }
    
}
