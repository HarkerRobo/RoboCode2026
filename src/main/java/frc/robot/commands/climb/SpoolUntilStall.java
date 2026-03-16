package frc.robot.commands.climb;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Climb;

public class SpoolUntilStall extends Command
{
    Timer timer = new Timer();
    public SpoolUntilStall()
    {
        addRequirements(Climb.getInstance());
    }

    @Override
    public void initialize()
    {
        timer.reset();
        Climb.getInstance().setSpoolingVoltage(Constants.Climb.SPOOL_VOLTAGE);
    }

    @Override
    public void execute()
    {
    }

    @Override
    public boolean isFinished()
    {
        return Climb.getInstance().isSpoolingStalling();
    }

    @Override
    public void end(boolean interrupted)
    {
        Climb.getInstance().setSpoolingVoltage(Volts.of(0.0));
    }
}
