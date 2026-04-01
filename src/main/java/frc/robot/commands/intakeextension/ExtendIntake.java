package frc.robot.commands.intakeextension;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.intake.IntakeExtension;

public class ExtendIntake extends Command
{
    /**
     * Claims subsys.
     */
    public ExtendIntake()
    {
        addRequirements(IntakeExtension.getInstance());
    }

    /**
     * Sets the intake extension motor to its extending voltage.
     * Begin extending the mechanism immediately.
     */
    @Override
    public void initialize()
    {
        IntakeExtension.getInstance().setVoltage(Constants.IntakeExtension.EXTENDING_VOLTAGE);
    }

    /**
     * Finishes when the intake stalls.
     */
    @Override
    public boolean isFinished()
    {
        return IntakeExtension.getInstance().isStalling();
    }

    /**
     * Applies the holding voltage once extension is complete.
     */
    @Override
    public void end(boolean interrupted)
    {
        IntakeExtension.getInstance().setVoltage(Constants.IntakeExtension.HOLDING_EXTEND_VOLTAGE);
    }
    
}
