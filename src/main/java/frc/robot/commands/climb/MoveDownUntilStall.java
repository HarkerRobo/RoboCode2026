package frc.robot.commands.climb;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Climb;

public class MoveDownUntilStall extends Command
{
    public MoveDownUntilStall ()
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
        Climb.getInstance().setElevatorVoltage(Constants.Climb.ELEVATOR_GO_DOWN_VOLTAGE);
    }

    @Override
    public boolean isFinished ()
    {
        return Climb.getInstance().isElevatorStalling();
    }

    @Override
    public void end (boolean interrupted)
    {
        Climb.getInstance().setElevatorVoltage((Constants.Climb.ELEVATOR_STAY_VOLTAGE));
    }

    @Override
    public String getName()
    {
        return "MoveDownUntilStall";
    }
}
