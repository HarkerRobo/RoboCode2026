package frc.robot.commands.climb;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Climb;

public class MoveDownUntilStall extends Command
{
    /**
     * Claims the Climb subsystem.
     */
    public MoveDownUntilStall()
    {
        addRequirements(Climb.getInstance());
    }

    /**
     * No setup required before running the command.
     * Method included for Command API completeness.
     */
    @Override
    public void initialize()
    {

    }

    /**
     * Drives the elevator downward at the configured voltage.
     * Executes every cycle until the stall detector trips.
     */
    @Override
    public void execute()
    {
        Climb.getInstance().setElevatorVoltage(Constants.Climb.ELEVATOR_GO_DOWN_VOLTAGE);
    }

    /**
     * Finishes when the elevator current crosses the stall threshold.
     * Uses the Climb subsystem’s stall detection.
     */
    @Override
    public boolean isFinished()
    {
        return Climb.getInstance().isElevatorStalling();
    }

    /**
     * Stops the elevator by setting voltage to zero.
     * Runs whether the command ends normally or is interrupted.
     */
    @Override
    public void end(boolean interrupted)
    {
        Climb.getInstance().setElevatorVoltage(Volts.of(0.0));
    }
}
