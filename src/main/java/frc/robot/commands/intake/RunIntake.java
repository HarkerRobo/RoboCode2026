package frc.robot.commands.intake;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.intake.Intake;

public class RunIntake extends Command 
{
    public RunIntake () 
    {
        addRequirements(Intake.getInstance());
    }

    @Override
    public void initialize()
    {
        Robot.instance.robotContainer.driver.setRumble(RumbleType.kBothRumble, 0.5);
        // Intake.getInstance().setVoltage(Volts.of(5.5));
        if (DriverStation.isAutonomous())
        {
            Intake.getInstance().setVelocity(RotationsPerSecond.of(Constants.Intake.INTAKE_VELOCITY_AUTON));
        }
        else
        {
            Intake.getInstance().setVelocity(RotationsPerSecond.of(Constants.Intake.INTAKE_VELOCITY));
        }
    }

    @Override
    public void execute () 
    {
    }

    @Override
    public boolean isFinished () 
    {
        return false;
    }

    @Override
    public void end (boolean interrupted) 
    {
        Robot.instance.robotContainer.driver.setRumble(RumbleType.kBothRumble, 0.0);
        Intake.getInstance().setVoltage(Volts.zero());
        // Intake.getInstance().setVelocity(RotationsPerSecond.zero());
    }
}
