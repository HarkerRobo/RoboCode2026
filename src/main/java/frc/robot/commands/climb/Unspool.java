package frc.robot.commands.climb;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Climb;

public class Unspool extends Command
{
    public Unspool()
    {
        addRequirements(Climb.getInstance());
    }

    @Override
    public void initialize()
    {
        Climb.getInstance().setSpoolingVoltage(Constants.Climb.UNSPOOL_VOLTAGE);
    }

    @Override
    public void execute()
    {
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }

    @Override
    public void end(boolean interrupted)
    {
        Climb.getInstance().setSpoolingVoltage((Volts.of(0.0)));
    }
}
