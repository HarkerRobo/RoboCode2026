package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.*;
import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Turret;

public class AimToAngle extends Command
{
    private double yaw; // degrees
    private double pitch; // degrees

    /**
     * 
     * @param yaw degrees
     * @param pitch degrees
    */
    public AimToAngle (double yaw, double pitch)
    {
        this.yaw = yaw;
        this.pitch = pitch;

        addRequirements(Turret.getInstance());
        addRequirements(Hood.getInstance());
    }

    @Override
    public void initialize ()
    {
        Turret.getInstance().setDesiredPosition(Degrees.of(yaw));
        Hood.getInstance().moveToPosition(Degrees.of(pitch));
        System.out.println("Aiming: (" + yaw + "°, " + pitch + "°)");
    }

    @Override
    public void execute ()
    {
    }

    @Override
    public boolean isFinished ()
    {
        return Turret.getInstance().readyToShoot() && Hood.getInstance().readyToShoot();
    }

    @Override
    public void end (boolean interrupted)
    {
        Hood.getInstance().moveToPosition(Hood.getInstance().getPosition());
    }
}
