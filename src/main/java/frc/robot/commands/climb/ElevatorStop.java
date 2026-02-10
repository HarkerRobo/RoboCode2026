package frc.robot.commands.climb;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Climb;

public class ElevatorStop extends Command
{
    public ElevatorStop ()
    {
        addRequirements(Climb.getInstance());
    }

    @Override
    public void initialize ()
    {
        
    }

    @Override
    public void execute ()
    {
        Climb.getInstance().setElevatorTargetPosition(Rotations.of(0.0));
    }

    @Override
    public boolean isFinished ()
    {
        return false;
    }

    @Override
    public void end (boolean interrupted)
    {
        
    }
}
