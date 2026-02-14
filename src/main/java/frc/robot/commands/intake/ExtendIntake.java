package frc.robot.commands.intake;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.intake.IntakeExtension;

public class ExtendIntake extends Command
{
    public ExtendIntake()
    {
        addRequirements(IntakeExtension.getInstance());
    }

    @Override
    public void initialize()
    {
        IntakeExtension.getInstance().setVoltage(Volts.of(Constants.IntakeExtension.EXTENDING_VOLTAGE));
    }

    @Override
    public void execute()
    {
    }

    @Override
    public boolean isFinished()
    {
        return IntakeExtension.getInstance().isStalling();
    }

    @Override
    public void end(boolean interrupted)
    {
        IntakeExtension.getInstance().setVoltage(Volts.of(0.0));
    }
}
