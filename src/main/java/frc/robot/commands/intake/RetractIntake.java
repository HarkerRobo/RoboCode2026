package frc.robot.commands.intake;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeExtension;

public class RetractIntake extends Command
{
    public RetractIntake()
    {
        addRequirements(IntakeExtension.getInstance());
    }

    @Override
    public void initialize()
    {
        IntakeExtension.getInstance().setExtensionVoltage(Volts.of(Constants.IntakeExtension.RETRACTING_VOLTAGE));
    }

    @Override
    public void execute()
    {
    }

    @Override
    public boolean isFinished()
    {
        return IntakeExtension.getInstance().extensionIsStalling();
    }

    @Override
    public void end(boolean interrupted)
    {
        IntakeExtension.getInstance().setExtensionVoltage(Volts.of(0.0));
    }
    
    @Override
    public String getName()
    {
        return "RetractIntake";
    }
}
