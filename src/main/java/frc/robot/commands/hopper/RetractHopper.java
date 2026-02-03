package frc.robot.commands.hopper;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.*;
import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Hopper;

public class RetractHopper extends Command
{
    public RetractHopper()
    {
        addRequirements(Hopper.getInstance());
    }

    @Override
    public void initialize ()
    {
        Hopper.getInstance().setVoltage(Volts.of(Constants.Hopper.BACKWARD_VOLTAGE));
    }

    @Override
    public void execute ()
    {

    }

    @Override
    public boolean isFinished ()
    {
        return Hopper.getInstance().isStalling();
    }

    @Override
    public void end (boolean interrupted)
    {
        
    }

    @Override
    public String getName()
    {
        return "Retract Hopper";
    }
}
