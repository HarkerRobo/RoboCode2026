package frc.robot.commands.hood;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Hood;

public class HoodManualUp extends Command
{
    public HoodManualUp()
    {
        addRequirements(Hood.getInstance());
    }
    
    @Override
    public void initialize()
    {
        Hood.getInstance().setVoltage(Volts.of(Constants.Hood.MANUAL_UP_VOLTAGE));
    }

    @Override
    public void execute()
    {
    }

    @Override
    public boolean isFinished()
    {
        return Hood.getInstance().isStalling();
    }

    @Override
    public void end(boolean interrupted)
    {
        Hood.getInstance().setVoltage(Volts.of(0.0));
        //Hood.getInstance().moveToPosition(Hood.getInstance().getPosition());
    }
}
