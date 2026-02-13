package frc.robot.commands.hood;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.*;
import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Hood;
import frc.robot.Constants;

public class ZeroHood extends Command
{
    public ZeroHood()
    {
        addRequirements(Hood.getInstance());
    }

    @Override
    public void initialize()
    {
    }

    @Override
    public void execute()
    {
        Hood.getInstance().moveToPosition(Degrees.of(Constants.Hood.MIN_ANGLE));
    }

    @Override
    public boolean isFinished()
    {
        return Hood.getInstance().isStalling();
    }

    @Override
    public void end(boolean interrupted)
    {
        Hood.getInstance().setPosition(Degrees.of(Constants.Hood.MIN_ANGLE));
    }
}
