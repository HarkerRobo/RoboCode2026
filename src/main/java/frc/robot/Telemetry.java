package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.*;
import static edu.wpi.first.units.Units.*;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructArrayTopic;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Simulation;
import frc.robot.simulation.SimulationState;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class Telemetry 
{
    private static Telemetry instance;

    private NetworkTableInstance tableInstance = NetworkTableInstance.getDefault();
    private NetworkTable table = tableInstance.getTable("1072");

    private NetworkTable turret = table.getSubTable("Turret");
    private StringPublisher turretCommand = turret.getStringTopic("command").publish();
    private DoublePublisher turretPosition = turret.getDoubleTopic("position (°)").publish();
    private DoublePublisher turretTargetPosition = turret.getDoubleTopic("target position (°)").publish();
    private DoublePublisher turretVelocity = turret.getDoubleTopic("velocity (° per s)").publish();
    private DoublePublisher turretVoltage = turret.getDoubleTopic("voltage (V)").publish();
    private BooleanPublisher turretReadyToShoot = turret.getBooleanTopic("ready to shoot?").publish();

    private NetworkTable hood = table.getSubTable("Hood");
    private StringPublisher hoodCommand = turret.getStringTopic("command").publish();
    private DoublePublisher hoodPosition = turret.getDoubleTopic("position (°)").publish();
    private DoublePublisher hoodTargetPosition = turret.getDoubleTopic("target position (°)").publish();
    private DoublePublisher hoodVelocity = turret.getDoubleTopic("velocity (° per s)").publish();
    private DoublePublisher hoodVoltage = turret.getDoubleTopic("voltage (V)").publish();
    private BooleanPublisher hoodReadyToShoot = turret.getBooleanTopic("ready to shoot?").publish();
    
    private NetworkTable shooter = table.getSubTable("Hood");
    private StringPublisher shooterCommand = turret.getStringTopic("command").publish();
    private DoublePublisher shooterVelocity = turret.getDoubleTopic("velocity (rot per s)").publish();
    private DoublePublisher shooterVoltage = turret.getDoubleTopic("voltage (V)").publish();
    private BooleanPublisher shooterReadyToShoot = turret.getBooleanTopic("ready to shoot?").publish();


    private NetworkTable persistent = table.getSubTable("[persistent variables]");
    // yaw is recorded so that we can record the position of the turret through power cycles without having to use a hard stop or otherwise zeroing
    private DoubleTopic turretYawRaw = persistent.getDoubleTopic("yaw");
    public DoubleEntry turretYawRawSubscriber = turretYawRaw.getEntry(0.0);
    private DoublePublisher turretYawRawPublisher = turretYawRaw.publish();

    private NetworkTable simulation = table.getSubTable("Simulation");
    private StructArrayPublisher<Translation3d> fuels = simulation.getStructArrayTopic("FuelPosition", Translation3d.struct).publish();
    private DoublePublisher fuelsInRobot = simulation.getDoubleTopic("Fuels in Robot").publish();
    private DoublePublisher fuelsInBlueHub = simulation.getDoubleTopic("Fuels in BlueHub").publish();
    private DoublePublisher fuelsInRedHub = simulation.getDoubleTopic("Fuels in RedHub").publish();
    private DoublePublisher fuelsInBlueOutpost = simulation.getDoubleTopic("Fuels in BlueOutpost").publish();
    private DoublePublisher fuelsInRedOutpost = simulation.getDoubleTopic("Fuels in RedOutpost").publish();
    private StructArrayPublisher<Translation3d> test = simulation.getStructArrayTopic("TEST", Translation3d.struct).publish();

    private Telemetry ()
    {
        turretYawRaw.setPersistent(true);
    }

    public void update ()
    {
        turretYawRaw.setPersistent(true);

        Command turretCommand = Turret.getInstance().getCurrentCommand();
        this.turretCommand.set(turretCommand == null ? "" : turretCommand.getName());
        turretPosition.set(Turret.getInstance().getPosition().in(Degrees));
        turretTargetPosition.set(Turret.getInstance().getDesiredPosition().in(Degrees));
        turretVelocity.set(Turret.getInstance().getVelocity().in(DegreesPerSecond));
        turretVoltage.set(Turret.getInstance().getVoltage().in(Volts));
        turretReadyToShoot.set(Turret.getInstance().readyToShoot());
        
        Command hoodCommand = Hood.getInstance().getCurrentCommand();
        this.hoodCommand.set(hoodCommand == null ? "" : hoodCommand.getName());
        hoodPosition.set(Hood.getInstance().getPosition().in(Degrees));
        hoodTargetPosition.set(Hood.getInstance().getDesiredPosition().in(Degrees));
        hoodVelocity.set(Hood.getInstance().getVelocity().in(DegreesPerSecond));
        hoodVoltage.set(Hood.getInstance().getVoltage().in(Volts));
        hoodReadyToShoot.set(Hood.getInstance().readyToShoot());

        Command shooterCommand = Shooter.getInstance().getCurrentCommand();
        this.shooterCommand.set(shooterCommand == null ? "" : shooterCommand.getName());
        shooterVelocity.set(Shooter.getInstance().getVelocity().in(RotationsPerSecond));
        shooterVoltage.set(Shooter.getInstance().getVoltage().in(Volts));
        shooterReadyToShoot.set(Shooter.getInstance().readyToShoot());

        turretYawRawPublisher.set(Turret.getInstance().getPosition().in(Rotations));

        fuels.set(SimulationState.getInstance().fuelPositionsRaw);

        fuelsInRobot.set(SimulationState.getInstance().fuelsInRobot);
        fuelsInBlueHub.set(SimulationState.getInstance().fuelsInBlueHub);
        fuelsInRedHub.set(SimulationState.getInstance().fuelsInRedHub);
        fuelsInBlueOutpost.set(SimulationState.getInstance().fuelsInBlueOutpost);
        fuelsInRedOutpost.set(SimulationState.getInstance().fuelsInRedOutpost);

        /*
        test.set(new Translation3d[] 
        {
            new Translation3d(Constants.Simulation.HUB_CONTENTS.getCenter().getX() - 0.5 * Constants.Simulation.HUB_CONTENTS.getXWidth(), 
                              Constants.Simulation.HUB_CONTENTS.getCenter().getY() - 0.5 * Constants.Simulation.HUB_CONTENTS.getYWidth(), 0.0),
            new Translation3d(Constants.Simulation.HUB_CONTENTS.getCenter().getX() + 0.5 * Constants.Simulation.HUB_CONTENTS.getXWidth(), 
                              Constants.Simulation.HUB_CONTENTS.getCenter().getY() + 0.5 * Constants.Simulation.HUB_CONTENTS.getYWidth(), 0.0),
            new Translation3d(Constants.Simulation.HUB_CONTENTS.getCenter().getX() - 0.5 * Constants.Simulation.HUB_CONTENTS.getXWidth(), 
                              Constants.Simulation.HUB_CONTENTS.getCenter().getY() + 0.5 * Constants.Simulation.HUB_CONTENTS.getYWidth(), 0.0),
            new Translation3d(Constants.Simulation.HUB_CONTENTS.getCenter().getX() + 0.5 * Constants.Simulation.HUB_CONTENTS.getXWidth(), 
                              Constants.Simulation.HUB_CONTENTS.getCenter().getY() - 0.5 * Constants.Simulation.HUB_CONTENTS.getYWidth(), 0.0)
        }
                              */
        test.set(new Translation3d[] 
        {
            new Translation3d(-0.84, 0.331, 0.075),
            new Translation3d(-0.0708, 1.008, 0.075)
        }
            );
    }

    public static Telemetry getInstance ()
    {
        if (instance == null) instance = new Telemetry();
        return instance;
    }
}
