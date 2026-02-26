package frc.robot.commands.climb;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Climb;

public class ElevatorUp extends Command
{
    /**
     * Claims the Climb subsystem.
     */
    public ElevatorUp()
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
     * Applies a constant positive voltage to raise the elevator.
     * Executes every cycle until the command is canceled.
     */
    @Override
    public void execute()
    {
        Climb.getInstance().setElevatorVoltage(Volts.of(2.0));
    }

    /**
     * This command never finishes on its own.
     * Must be interrupted or canceled externally.
     */
    @Override
    public boolean isFinished()
    {
        return false;
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
