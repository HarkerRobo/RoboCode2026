package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class IntakeBall extends Command {
    public IntakeBall () {
        addRequirements(Intake.getInstance());
    }

    @Override
    public void initialize()
    {

    }

    @Override
    public void execute () {
        Intake.getInstance().setVelocity(Constants.Intake.INTAKE_VELOCITY);
    }

    @Override
    public boolean isFinished () {
        return false;
    }

    @Override
    public void end (boolean interrupted) {
        Intake.getInstance().setVelocity(Constants.Intake.DEFAULT_INTAKE_VELOCITY);
    }
}
