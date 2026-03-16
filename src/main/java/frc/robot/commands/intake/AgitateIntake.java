package frc.robot.commands.intake;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.intake.Intake;

public class AgitateIntake extends Command
{
    Timer timer = new Timer();
    public  AgitateIntake()
    {
        addRequirements(Intake.getInstance());
    }

    @Override
    public void initialize()
    {
        timer.reset();
    }

    @Override
    public void execute()
    {
        Intake.getInstance().setVoltage(Volts.of(
            Math.sin(2 * Math.PI / Constants.IntakeExtension.AGITATE_PERIOD_SECONDS * timer.get())
            * (Constants.IntakeExtension.AGITATE_MAX_VOLTAGE - Constants.IntakeExtension.AGITATE_MIN_VOLTAGE) + 
            Constants.IntakeExtension.AGITATE_MIN_VOLTAGE));
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }

    @Override
    public void end(boolean interrupted)
    {
        Intake.getInstance().setVoltage(Volts.zero());
    }
}
