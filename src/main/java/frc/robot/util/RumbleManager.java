package frc.robot.util;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import frc.robot.Robot;

public class RumbleManager 
{
    private static RumbleManager instance;

    private double driverRumble = 0.0;


    public void setDriverRumble(double rumbleValue)
    {
        driverRumble = rumbleValue;
    }

    public void periodic()
    {
        Robot.instance.robotContainer.driver.setRumble(RumbleType.kBothRumble, driverRumble);
    }

    public static RumbleManager getInstance()
    {
        if (instance == null) instance = new RumbleManager();
        return instance;
    }
}

