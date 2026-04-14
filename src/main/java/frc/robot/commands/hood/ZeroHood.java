package frc.robot.commands.hood;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Hood;

public class ZeroHood extends Command
{
    Timer timer = new Timer();

    /**
     * Claims the Hood subsystem.
     */
    public ZeroHood()
    {
        addRequirements(Hood.getInstance());
    }

    /**
     * Commands the hood toward its minimum allowed angle.
     * Uses the subsystem’s internal Motion Magic control.
     */
    @Override
    public void initialize()
    {
        timer.reset();
        Hood.getInstance().setVoltage(Volts.of(Constants.Hood.ZEROING_VOLTAGE));
    }

    @Override
    public void execute()
    {
        System.out.println("Zeroing hood");
    }

    /**
     * Finishes when the hood stalls or reaches the target.
     * Either condition indicates the hood is at its mechanical zero.
     */
    @Override
    public boolean isFinished()
    {
        return Hood.getInstance().isStalling();
    }

    /**
     * If the command completed normally, resets the hood’s internal position.
     * Ensures the subsystem’s encoder matches the mechanical zero point.
     */
    @Override
    public void end(boolean interrupted)
    {
        if (!interrupted)
        {
            Hood.getInstance().setPosition(Degrees.of(Constants.Hood.ZEROING_POSITION));
        }
        Hood.getInstance().setVoltage(Volts.of(0.0));
    }
}
