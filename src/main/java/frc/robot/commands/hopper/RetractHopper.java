package frc.robot.commands.hopper;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Hopper;

public class RetractHopper extends Command
{
    /**
     * Claims the Hopper subsystem.
     */
    public RetractHopper()
    {
        addRequirements(Hopper.getInstance());
    }

    /**
     * Applies the backward voltage to retract the hopper.
     */
    @Override
    public void initialize()
    {
        Hopper.getInstance().setVoltage(Volts.of(Constants.Hopper.BACKWARD_VOLTAGE));
    }

    /**
     * No repeated action is required.
     */
    @Override
    public void execute()
    {

    }

    /**
     * Finishes when the hopper stalls against its mechanical stop.
     */
    @Override
    public boolean isFinished()
    {
        return Hopper.getInstance().isStalling();
    }

    /**
     * No cleanup is required.
     */
    @Override
    public void end(boolean interrupted)
    {

    }
}
