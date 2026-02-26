package frc.robot.commands.hood;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Hood;

public class ZeroHoodSoft extends Command
{
    /**
     * Claims the Hood subsystem.
     */
    public ZeroHoodSoft()
    {
        addRequirements(Hood.getInstance());
    }

    /**
     * Applies the configured zeroing voltage to move the hood downward.
     */
    @Override
    public void initialize()
    {
        Hood.getInstance().setVoltage(Volts.of(Constants.Hood.ZEROING_VOLTAGE));
    }

    /**
     * Finishes when the hood current indicates a stall.
     * Stall implies the hood has reached its mechanical zero point.
     */
    @Override
    public boolean isFinished()
    {
        return Hood.getInstance().isStalling();
    }

    /**
     * If the command completed normally, resets the hood’s internal encoder.
     * Ensures the subsystem’s position matches the mechanical zero.
     */
    @Override
    public void end(boolean interrupted)
    {
        if (!interrupted)
        {
            System.out.println("Soft zero done");
            Hood.getInstance().setPosition(Degrees.of(Constants.Hood.MIN_ANGLE));
        }
    }
}
