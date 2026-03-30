package frc.robot.commands.intakeextension;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.intake.IntakeExtension;

public class IntakeExtensionManual extends Command
{
    boolean stallingForward;
    boolean stallingBack;
    /**
     * Claims subsystem.
     */
    public IntakeExtensionManual()
    {
        addRequirements(IntakeExtension.getInstance());
    }

    /**
     * Applies manual extend or retract voltage based on operator input.
     * Monitors stall conditions and switches to holding voltages when necessary.
     */
    @Override
    public void execute()
    {
        if (Robot.instance.robotContainer.operator.b().getAsBoolean() && !stallingForward)
        {
            IntakeExtension.getInstance().setVoltage(Volts.of(Constants.IntakeExtension.EXTENDING_VOLTAGE));
        }
        else if (Robot.instance.robotContainer.operator.x().getAsBoolean() && !stallingBack)
        {
            IntakeExtension.getInstance().setVoltage(Volts.of(Constants.IntakeExtension.RETRACTING_VOLTAGE));
        }

        if (IntakeExtension.getInstance().getVoltage().in(Volts) > 0.0)
        {
            if (IntakeExtension.getInstance().isStalling())
            {
                IntakeExtension.getInstance().setVoltage(Volts.of(Constants.IntakeExtension.HOLDING_EXTEND_VOLTAGE));
                stallingForward = true;
            }
            stallingBack = false;
        }
        else if (IntakeExtension.getInstance().getVoltage().in(Volts) < 0.0)
        {
            if (IntakeExtension.getInstance().isStalling())
            {
                stallingBack = true;
                IntakeExtension.getInstance().setVoltage(Volts.of(Constants.IntakeExtension.HOLDING_RETRACT_VOLTAGE));
            }
            stallingForward = false;
        }
        else
        {
            stallingForward = false;
            stallingBack = false;
        }
    }

    /**
     * Never finishes.
     */
    @Override
    public boolean isFinished()
    {
        return false;
    }
}
