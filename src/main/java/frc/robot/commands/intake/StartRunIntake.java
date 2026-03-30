package frc.robot.commands.intake;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.intake.Intake;

public class StartRunIntake extends Command 
{
    /**
     * Claims subsys.
     */
    public StartRunIntake () 
    {
        addRequirements(Intake.getInstance());
    }

    /**
    * Activates controller rumble feedback and sets the intake to the correct speed.
    * Chooses between autonomous and teleop velocities based on robot mode.
    */
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


    /**
     * Command finishes immediately.
     */
    @Override
    public boolean isFinished () 
    {
        return true;
    }

    /**
     * Nada.
     */
    @Override
    public void end (boolean interrupted) 
    {
    }
}
