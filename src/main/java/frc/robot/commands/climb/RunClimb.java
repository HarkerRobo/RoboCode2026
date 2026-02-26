package frc.robot.commands.climb;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Climb;

public class RunClimb extends Command
{
    Timer timer = new Timer();

    /**
     * Claims the Climb subsystem.
     */
    public RunClimb()
    {
        addRequirements(Climb.getInstance());
    }

    /**
     * Resets the timer and applies the climb deployment voltage.
     * Starts the timed climb motion.
     */
    @Override
    public void initialize()
    {
        timer.reset();
        Climb.getInstance().setClimbVoltage(Constants.Climb.CLIMB_DEPLOY_VOLTAGE);
    }

    /**
     * No repeated action required during execution.
     * Voltage is held from initialize() until the timer expires.
     */
    @Override
    public void execute()
    {
    }

    /**
     * Finishes once the configured time has elapsed.
     * Uses a 5‑second timeout to end the command.
     */
    @Override
    public boolean isFinished()
    {
        return timer.hasElapsed(5.0);
    }

    /**
     * Stops the climb motor by setting voltage to zero.
     * Runs whether the command ends normally or is interrupted.
     */
    @Override
    public void end(boolean interrupted)
    {
        Climb.getInstance().setClimbVoltage(Volts.of(0.0));
    }
}
