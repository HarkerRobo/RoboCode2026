package frc.robot.util;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

    /**
     * Wraps a command so it schedules itself independently of the caller I think???.
     * Trigger a command without blocking or chaining it to anything else???? maybe??
     */
public class IndependentCommand extends SequentialCommandGroup
{
    public IndependentCommand(Command command)
    {
        addCommands(Commands.runOnce(()->CommandScheduler.getInstance().schedule(command)));
    }
}
