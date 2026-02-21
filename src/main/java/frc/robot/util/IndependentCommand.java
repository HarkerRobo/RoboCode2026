package frc.robot.util;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class IndependentCommand extends SequentialCommandGroup
{
    public IndependentCommand(Command command)
    {
        addCommands(Commands.runOnce(()->CommandScheduler.getInstance().schedule(command)));
    }
}
