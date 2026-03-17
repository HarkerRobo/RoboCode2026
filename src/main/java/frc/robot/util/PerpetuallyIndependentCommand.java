package frc.robot.util;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class PerpetuallyIndependentCommand extends Command
{
    Command command;

    public PerpetuallyIndependentCommand(Command command)
    {
        this.command = command;
    }

    public void initialize()
    {
        CommandScheduler.getInstance().schedule(command);
    }

    public void execute()
    {
        if (!command.isScheduled()) CommandScheduler.getInstance().schedule(command);
    }

    public boolean isFinished()
    {
        return false;
    }

    public void end (boolean interrupted)
    {
        command.cancel();
    }
}
