package frc.robot.commands.climb;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Climb;

public class ClimbToLevel extends Command
{
    int level;
    Angle targetAngle;

    /**
     * Creates a climb command that moves the elevator to a preset level.
     * Stores the level index and claims the Climb subsystem.
     */
    public ClimbToLevel(int level)
    {
        addRequirements(Climb.getInstance());
        this.level = level;
    }

    /**
     * Selects the target angle based on the requested level.
     * Defaults to zero rotations if the level is invalid.
     */
    @Override
    public void initialize()
    {
        targetAngle = switch (level) {
            case 1 -> Constants.Climb.CLIMB_POSITION_LEVEL_1;
            case 2 -> Constants.Climb.CLIMB_POSITION_LEVEL_2;
            case 3 -> Constants.Climb.CLIMB_POSITION_LEVEL_3;
            default -> Rotations.zero();
        };
    }

    /**
     * Sends the target elevator position to the Climb subsystem.
     * Runs every cycle until the elevator reaches the target.
     */
    @Override
    public void execute()
    {
        Climb.getInstance().setElevatorTargetPosition(targetAngle);
        //Climb.getInstance().setElevatorVoltage(Volts.of(
            //Math.min(1.0,Math.abs((levelHeights[level].minus(Climb.getInstance().getElevatorPosition())).times(5.0).in(Rotations)))
            //* Math.signum(levelHeights[level].minus(Climb.getInstance().getElevatorPosition()).in(Rotations))));
    }

    /**
     * Finishes when the elevator is within the allowed tolerance of the target.
     * Uses the isNear check on the position.
     */
    @Override
    public boolean isFinished()
    {
        return Climb.getInstance()
                .getElevatorPosition()
                .isNear(targetAngle, Rotations.of(Constants.EPSILON));
    }

    /**
     * No cleanup required when the command ends.
     * Method kept for consistency with Command API.
     */
    @Override
    public void end(boolean interrupted)
    {
        
    }

    /**
     * Returns a readable name including the target level.
     * debugging and command tracing.
     */
    @Override
    public String getName()
    {
        return "ClimbToLevel[" + level + "]";
    }
}
