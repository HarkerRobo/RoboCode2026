package frc.robot.commands.intake;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.intake.Intake;

public class StartEjectIntake extends Command 
{
    public StartEjectIntake () 
    {
        addRequirements(Intake.getInstance());
    }

    @Override
    public void initialize()
    {
        Robot.instance.robotContainer.driver.setRumble(RumbleType.kBothRumble, 0.0);
        // Intake.getInstance().setVoltage(Volts.of(-2.5));
        Intake.getInstance().setVelocity(RotationsPerSecond.of(Constants.Intake.EJECT_VELOCITY));
    }

    @Override
    public boolean isFinished () 
    {
        return true;
    }
}
