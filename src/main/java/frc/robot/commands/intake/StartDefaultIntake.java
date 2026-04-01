package frc.robot.commands.intake;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.intake.Intake;

/**
 * is still used but applies a zero voltage
 */
public class StartDefaultIntake extends Command 
{
    /**
     * Claims subsystem.
     */
    public StartDefaultIntake () 
    {
        addRequirements(Intake.getInstance());
    }

    /**
     * Clears controller rumble feedback and sets the intake voltage to zero.
     * Runs once to place the subsystem into its default inactive state.
     */
    @Override
    public void initialize()
    {
        Robot.instance.robotContainer.driver.setRumble(RumbleType.kBothRumble, 0.0);
        Intake.getInstance().setVoltage(Volts.zero());
        // Intake.getInstance().setVelocity(RotationsPerSecond.zero());
    }

    /**
     * Command finishes immediately.
     */
    @Override
    public boolean isFinished () 
    {
        return true;
    }
}
