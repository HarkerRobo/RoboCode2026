package frc.robot;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret;

public class Telemetry 
{
    private static Telemetry instance;

    private NetworkTableInstance tableInstance = NetworkTableInstance.getDefault();
    private NetworkTable table = tableInstance.getTable("1072");

    private NetworkTable turret = table.getSubTable("Turret");
    private StringPublisher turretCommand = turret.getStringTopic("command").publish();
    private DoublePublisher turretYaw = turret.getDoubleTopic("yaw °").publish();
    private DoublePublisher turretPitch = turret.getDoubleTopic("pitch °").publish();

    private NetworkTable persistent = table.getSubTable("[persistent variables]");
    // yaw is recorded so that we can record the position of the turret through power cycles without having to use a hard stop or otherwise zeroing
    private DoubleTopic turretYawRaw = persistent.getDoubleTopic("yaw");
    public DoubleEntry turretYawRawSubscriber = turretYawRaw.getEntry(0.0);
    private DoublePublisher turretYawRawPublisher = turretYawRaw.publish();

    private Telemetry ()
    {
        turretYawRaw.setPersistent(true);
    }

    public void update ()
    {
        turretYawRaw.setPersistent(true);

        Command turretCommand = Turret.getInstance().getCurrentCommand();
        this.turretCommand.set(turretCommand == null ? "" : turretCommand.getName());

        turretYaw.set(Turret.getInstance().getYaw());
        turretPitch.set(Turret.getInstance().getPitch());

        turretYawRawPublisher.set(Turret.getInstance().getRawYaw());
    }

    public static Telemetry getInstance ()
    {
        if (instance == null) instance = new Telemetry();
        return instance;
    }
}
