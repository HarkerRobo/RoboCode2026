package frc.robot.commands.hood;

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
        Hood.getInstance().moveToPosition(Degrees.of(Constants.Hood.MIN_ANGLE));
    }

    @Override
    public void execute()
    {
    }

    @Override
    public boolean isFinished()
    {
        return Hood.getInstance().isStalling() || Hood.getInstance().readyToShoot();
    }

    @Override
    public void end(boolean interrupted)
    {
        if (!interrupted)
        {
            System.out.println("Zeroing Hood");
            Hood.getInstance().setPosition(Degrees.of(Constants.Hood.MIN_ANGLE));
        }
    }
}
