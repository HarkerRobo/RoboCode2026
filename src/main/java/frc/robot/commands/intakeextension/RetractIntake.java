package frc.robot.commands.intakeextension;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.intake.IntakeExtension;

public class RetractIntake extends Command
{
    /**
     * Claims subsys.
     */
    public RetractIntake()
    {
        addRequirements(IntakeExtension.getInstance());
    }

    /**
     * Sets the intake extension motor to its retracting voltage.
     */
    @Override
    public void initialize()
    {
        IntakeExtension.getInstance().setVoltage(Volts.of(Constants.IntakeExtension.RETRACTING_VOLTAGE));
    }

    /**
     * Finishes when the Intake stalls.
     */
    @Override
    public boolean isFinished()
    {
        return IntakeExtension.getInstance().isStalling();
    }

    /**
     * Applies the holding voltage once retraction is complete.
     */
    @Override
    public void end(boolean interrupted)
    {
        IntakeExtension.getInstance().setVoltage(Volts.of(Constants.IntakeExtension.HOLDING_RETRACT_VOLTAGE));
    }
    
}
