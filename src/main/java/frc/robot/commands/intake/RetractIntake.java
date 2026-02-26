package frc.robot.commands.intake;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.intake.IntakeExtension;

public class RetractIntake extends Command
{
    /**
     * Claims the IntakeExtension subsystem.
     */
    public RetractIntake()
    {
        addRequirements(IntakeExtension.getInstance());
    }

    /**
     * Applies the configured retracting voltage.
     */
    @Override
    public void initialize()
    {
        IntakeExtension.getInstance().setVoltage(Volts.of(Constants.IntakeExtension.RETRACTING_VOLTAGE));
    }

    /**
     * No repeated action is required.
     */
    @Override
    public void execute()
    {
    }

    /**
     * Finishes when the intake extension stalls against its mechanical stop.
     */
    @Override
    public boolean isFinished()
    {
        return IntakeExtension.getInstance().isStalling();
    }

    /**
     * Sets the extension voltage to zero when the command ends.
     */
    @Override
    public void end(boolean interrupted)
    {
        IntakeExtension.getInstance().setVoltage(Volts.of(0.0));
    }
}
