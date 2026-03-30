package frc.robot.commands.hood;

import static edu.wpi.first.units.Units.Volts;

import javax.lang.model.util.ElementScanner14;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Hood;

public class HoodManual extends Command
{
    private boolean stallingUp = false;
    private boolean stallingDown = false;

    /**
     * Claims the Hood subsystem for manual voltage control.
     * Tracks stall conditions to prevent overdriving the mechanism.
     */
    public HoodManual()
    {
        addRequirements(Hood.getInstance());
    }
 
    /**
     * Nothing.
     */
    @Override
    public void initialize()
    {
    }

    /**
     * /**
     * Applies manual up or down voltage based on operator input.
     * Tracks stall direction to avoid driving to limit.
     */
    @Override
    public void execute()
    {
        if (Robot.instance.robotContainer.operator.y().getAsBoolean() && !stallingUp)
        {
            Hood.getInstance().setVoltage(Volts.of(Constants.Hood.MANUAL_UP_VOLTAGE));
        }
        else if (Robot.instance.robotContainer.operator.a().getAsBoolean() && !stallingDown)
        {
            Hood.getInstance().setVoltage(Volts.of(Constants.Hood.MANUAL_DOWN_VOLTAGE));
        }

        if (Hood.getInstance().getVoltage().in(Volts) < 0.0)
        {
            if (Hood.getInstance().isStalling()) stallingUp = true;
            stallingDown = false;
        }
        else if (Hood.getInstance().getVoltage().in(Volts) > 0.0)
        {
            if (Hood.getInstance().isStalling()) stallingDown = true;
            stallingUp = false;
        }
        else
        {
            stallingUp = false;
            stallingDown = false;
        }
    }

    /**
     * Command doesn't finish
     * Innteruppt externally.
     */
    @Override
    public boolean isFinished()
    {
        return false;
    }


    /**
     * Nada.
     */
    @Override
    public void end(boolean interrupted)
    {
    }
}
