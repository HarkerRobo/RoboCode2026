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
    Angle[] levelHeights;
    public ClimbToLevel (int level)
    {
        addRequirements(Climb.getInstance());
        this.level=level;
        this.levelHeights = new Angle[] {Constants.Climb.CLIMB_POSITION_LEVEL_1, Constants.Climb.CLIMB_POSITION_LEVEL_2, Constants.Climb.CLIMB_POSITION_LEVEL_3};
    }

    @Override
    public void initialize ()
    {
        Climb.getInstance().setElevatorTargetPosition(levelHeights[level]);
    }

    @Override
    public void execute ()
    {
        //Climb.getInstance().setElevatorVoltage(Volts.of(
            //Math.min(1.0,Math.abs((levelHeights[level].minus(Climb.getInstance().getElevatorPosition())).times(5.0).in(Rotations)))
            //* Math.signum(levelHeights[level].minus(Climb.getInstance().getElevatorPosition()).in(Rotations))));
    }

    @Override
    public boolean isFinished ()
    {
        return Climb.getInstance().getElevatorPosition().isNear(levelHeights[level], Rotations.of(Constants.EPSILON));
    }

    @Override
    public void end (boolean interrupted)
    {
        
    }
}
