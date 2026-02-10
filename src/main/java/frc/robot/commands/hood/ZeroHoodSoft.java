package frc.robot.commands.hood;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Hood;

public class ZeroHoodSoft extends Command
{
    public ZeroHoodSoft()
    {
        addRequirements(Hood.getInstance());
    }

    @Override
    public void initialize()
    {
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
        Hood.getInstance().setVoltage(Volts.of(Constants.Hood.KG));

        if (!interrupted)
        {
            Hood.getInstance().setPosition(Degrees.of(Constants.Hood.MIN_ANGLE));
        }
    }
    
    @Override
    public String getName()
    {
        return "ZeroHoodSoft";
    }
}
