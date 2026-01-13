package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
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
    }

    @Override
    public void initialize ()
    {
        Turret.getInstance().setTargetYaw(yaw);
        Turret.getInstance().setTargetPitch(pitch);
        System.out.println("Aiming: (" + yaw + "°, " + pitch + "°)");
    }

    @Override
    public void execute ()
    {
    }

    @Override
    public boolean isFinished ()
    {
        return Math.abs(Turret.getInstance().getYaw() - yaw) <= Constants.Turret.ERROR_THRESHOLD &&
            Math.abs(Turret.getInstance().getPitch() - pitch) <= Constants.Turret.ERROR_THRESHOLD;
    }

    @Override
    public void end (boolean interrupted)
    {
        Turret.getInstance().maintainPitch();
    }
}
