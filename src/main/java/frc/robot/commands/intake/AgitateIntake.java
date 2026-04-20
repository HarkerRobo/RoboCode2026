package frc.robot.commands.intake;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.intake.Intake;

public class AgitateIntake extends Command
{
    Timer timer = new Timer();
    /**
     * Claims the Intake subsystem.
     */
    public  AgitateIntake()
    {
        addRequirements(Intake.getInstance());
    }

    /**
     * Resets the internal timer.
     */
    @Override
    public void initialize()
    {
        timer.reset();
        timer.start();
    }

    @Override
    /**
     * Applies a sinusoidal voltage to oscillate the intake.
     * Produces continuous agitation during operation.
     */
    public void execute()
    {
        System.out.printf("Time: %f\n", timer.get());
        Intake.getInstance().setVoltage(Volts.of(
            (Math.sin(2 * Math.PI / Constants.IntakeExtension.AGITATE_PERIOD_SECONDS.in(Seconds) * timer.get()) + 1.0)
            * (Constants.IntakeExtension.AGITATE_MAX_VOLTAGE.minus(Constants.IntakeExtension.AGITATE_MIN_VOLTAGE).in(Volts)) / 2.0 + 
            Constants.IntakeExtension.AGITATE_MIN_VOLTAGE.in(Volts)));
    }

    /**
     * This command never finishes on its own.
     */
    @Override
    public boolean isFinished()
    {
        return false;
    }

    /**
     * Stops the intake by applying zero volts.
     */
    @Override
    public void end(boolean interrupted)
    {
        Intake.getInstance().setVoltage(Volts.zero());
    }
}
