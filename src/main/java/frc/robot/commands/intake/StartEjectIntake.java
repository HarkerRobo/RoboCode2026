package frc.robot.commands.intake;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.intake.Intake;
import frc.robot.util.RumbleManager;

public class StartEjectIntake extends Command 
{
    /**
     * Claims subsystem.
     */
    public StartEjectIntake () 
    {
        addRequirements(Intake.getInstance());
    }

    /**
     * Clears controller rumble feedback and sets the intake to eject speed.
     * Runs once to begin reversing the intake immediately.
     */
    @Override
    public void initialize()
    {
        RumbleManager.getInstance().setDriverRumble(0.0);
        // Intake.getInstance().setVoltage(Volts.of(-2.5));
        Intake.getInstance().setVelocity(RotationsPerSecond.of(Constants.Intake.EJECT_VELOCITY));
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
