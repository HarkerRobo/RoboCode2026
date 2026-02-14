package frc.robot.commands.climb;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Climb;

public class UndeployClimb extends Command
{
    Timer timer = new Timer();
    public UndeployClimb()
    {
        addRequirements(Climb.getInstance());
    }

    @Override
    public void initialize()
    {
        timer.reset();
        Climb.getInstance().setClimbVoltage(Constants.Climb.CLIMB_UNDEPLOY_VOLTAGE);
    }

    @Override
    public void execute()
    {
    }

    @Override
    public boolean isFinished()
    {
        return timer.hasElapsed(5.0);
    }

    @Override
    public void end(boolean interrupted)
    {
        Climb.getInstance().setClimbVoltage(Volts.of(0.0));
    }
}
