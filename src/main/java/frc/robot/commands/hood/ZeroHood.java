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

    public ZeroHood()
    {
        addRequirements(Hood.getInstance());
    }

    @Override
    public void initialize()
    {
        timer.reset();
        Hood.getInstance().setVoltage(Volts.of(Constants.Hood.ZEROING_VOLTAGE));
    }

    @Override
    public boolean isFinished()
    {
        return Hood.getInstance().isStalling();
    }

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
