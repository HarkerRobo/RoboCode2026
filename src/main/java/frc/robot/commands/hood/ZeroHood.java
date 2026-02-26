package frc.robot.commands.hood;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Hood;
import frc.robot.Constants;

public class ZeroHood extends Command
{
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
        Hood.getInstance().moveToPosition(Degrees.of(Constants.Hood.MIN_ANGLE));
    }

    /**
     * No repeated action required during execution.
     */
    @Override
    public void execute()
    {
    }

    /**
     * Finishes when the hood stalls or reaches the target.
     * Either condition indicates the hood is at its mechanical zero.
     */
    @Override
    public boolean isFinished()
    {
        return Hood.getInstance().isStalling() || Hood.getInstance().readyToShoot();
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
            System.out.println("Zeroing Hood");
            Hood.getInstance().setPosition(Degrees.of(Constants.Hood.MIN_ANGLE));
        }
    }
}
