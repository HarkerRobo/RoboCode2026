package frc.robot.commands.hood;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Hood;

public class HoodManualDown extends Command
{
    public HoodManualDown()
    {
        addRequirements(Hood.getInstance());
    }
    
    @Override
    public void initialize()
    {
        Hood.getInstance().setVoltage(Volts.of(Constants.Hood.MANUAL_DOWN_VOLTAGE));
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
        Hood.getInstance().moveToPosition(Hood.getInstance().getPosition());
    }
}
