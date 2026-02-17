package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.simulation.SimulationState;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterIndexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeExtension;
import frc.robot.subsystems.Climb;

public class Telemetry 
{
    private static Telemetry instance;

    private NetworkTableInstance tableInstance = NetworkTableInstance.getDefault();
    private NetworkTable table = tableInstance.getTable("1072");

    private StringPublisher mostRecentAim = table.getStringTopic("Most Recent Aim").publish();
    private DoublePublisher hoodOffset = table.getDoubleTopic("Hood Offset").publish();

    private NetworkTable intake = table.getSubTable("Intake");
    private StringPublisher intakeCommand = intake.getStringTopic("main command").publish();
    private DoublePublisher intakeMainVelocity = intake.getDoubleTopic("main velocity (rot per s)").publish();
    private DoublePublisher intakeMainVoltage = intake.getDoubleTopic("main voltage (V)").publish();

    private StringPublisher intakeExtensionCommand = intake.getStringTopic("extension command").publish();
    private DoublePublisher intakeExtensionPosition = intake.getDoubleTopic("extension position (rot)").publish();
    private DoublePublisher intakeExtensionVelocity = intake.getDoubleTopic("extension velocity (rot per s)").publish();
    private DoublePublisher intakeExtensionVoltage = intake.getDoubleTopic("extension voltage (V)").publish();

    /*
    private NetworkTable turret = table.getSubTable("Turret");
    private StringPublisher turretCommand = turret.getStringTopic("command").publish();
    private DoublePublisher turretPosition = turret.getDoubleTopic("position (°)").publish();
    private DoublePublisher turretTargetPosition = turret.getDoubleTopic("target position (°)").publish();
    private DoublePublisher turretVelocity = turret.getDoubleTopic("velocity (° per s)").publish();
    private DoublePublisher turretVoltage = turret.getDoubleTopic("voltage (V)").publish();
    private BooleanPublisher turretReadyToShoot = turret.getBooleanTopic("ready to shoot?").publish();
    */

    private NetworkTable hood = table.getSubTable("Hood");
    private StringPublisher hoodCommand = hood.getStringTopic("command").publish();
    private DoublePublisher hoodPosition = hood.getDoubleTopic("position (°)").publish();
    private DoublePublisher hoodTargetPosition = hood.getDoubleTopic("target position (°)").publish();
    private DoublePublisher hoodVelocity = hood.getDoubleTopic("velocity (° per s)").publish();
    private DoublePublisher hoodVoltage = hood.getDoubleTopic("voltage (V)").publish();
    private BooleanPublisher hoodReadyToShoot = hood.getBooleanTopic("ready to shoot?").publish();
    
    private NetworkTable shooter = table.getSubTable("Shooter");
    private StringPublisher shooterCommand = shooter.getStringTopic("command").publish();
    private DoublePublisher shooterLeftVelocity = shooter.getDoubleTopic("left velocity (rot per s)").publish();
    private DoublePublisher shooterLeftVoltage = shooter.getDoubleTopic("left voltage (V)").publish();
    private DoublePublisher shooterRightVelocity = shooter.getDoubleTopic("right velocity (rot per s)").publish();
    private DoublePublisher shooterRightVoltage = shooter.getDoubleTopic("right voltage (V)").publish();

    private NetworkTable climb = table.getSubTable("Climb");
    private StringPublisher climbCommand = climb.getStringTopic("command").publish();
    private DoublePublisher climbElevatorVelocity = climb.getDoubleTopic("elevator velocity (rot per s)").publish();
    private DoublePublisher climbElevatorVoltage = climb.getDoubleTopic("elevator voltage (V)").publish();
    private DoublePublisher climbElevatorPosition = climb.getDoubleTopic("elevator position (rot)").publish();
    private DoublePublisher climbElevatorTarget = climb.getDoubleTopic("elevator target (rot)").publish();
    private DoublePublisher climbClimbVoltage = climb.getDoubleTopic("climb voltage (V)").publish();

    /*
    private NetworkTable persistent = table.getSubTable("[persistent variables]");
    // yaw is recorded so that we can record the position of the turret through power cycles without having to use a hard stop or otherwise zeroing
    private DoubleTopic turretYawRaw = persistent.getDoubleTopic("yaw");
    public DoubleEntry turretYawRawSubscriber = turretYawRaw.getEntry(0.0);
    private DoublePublisher turretYawRawPublisher = turretYawRaw.publish();
    */

    private NetworkTable simulation = table.getSubTable("Simulation");
    private StructArrayPublisher<Translation3d> fuels = simulation.getStructArrayTopic("FuelPosition", Translation3d.struct).publish();
    private IntegerPublisher fuelsInRobot = simulation.getIntegerTopic("Fuels in Robot").publish();
    private IntegerPublisher fuelsInBlueHub = simulation.getIntegerTopic("Fuels in BlueHub").publish();
    private IntegerPublisher fuelsInRedHub = simulation.getIntegerTopic("Fuels in RedHub").publish();
    private IntegerPublisher fuelsInBlueOutpost = simulation.getIntegerTopic("Fuels in BlueOutpost").publish();
    private IntegerPublisher fuelsInRedOutpost = simulation.getIntegerTopic("Fuels in RedOutpost").publish();
    private StructArrayPublisher<Translation3d> test = simulation.getStructArrayTopic("TEST", Translation3d.struct).publish();

    private NetworkTable indexer = table.getSubTable("Indexer");
    private StringPublisher indexerCommand = indexer.getStringTopic("command").publish();
    private DoublePublisher indexerVelocity = indexer.getDoubleTopic("velocity (rps)").publish();
    private DoublePublisher indexerVoltage = indexer.getDoubleTopic("voltage (V)").publish();
    
    private NetworkTable hopper = table.getSubTable("Hopper");
    private StringPublisher hopperCommand = hopper.getStringTopic("command").publish();
    private DoublePublisher hopperVelocity = hopper.getDoubleTopic("velocity (rot per s)").publish();
    private DoublePublisher hopperVoltage = hopper.getDoubleTopic("voltage (V)").publish();
    private DoublePublisher hopperPosition = hopper.getDoubleTopic("current position (rotations)").publish();
    // private DoublePublisher hopperTarget = hopper.getDoubleTopic("target position (rotations)").publish();

    private NetworkTable shooterIndexer = table.getSubTable("ShooterIndexer");
    private StringPublisher shooterIndexerCommand = shooterIndexer.getStringTopic("command").publish();
    private DoublePublisher shooterIndexerVelocity = shooterIndexer.getDoubleTopic("velocity (rps)").publish();
    private DoublePublisher shooterIndexerVoltage = shooterIndexer.getDoubleTopic("voltage (V)").publish();
    

    /* Robot swerve drive state */
    private final NetworkTable driveStateTable = tableInstance.getTable("DriveState");
    private final StructPublisher<Pose2d> drivePose = driveStateTable.getStructTopic("Pose", Pose2d.struct).publish();
    private final StructPublisher<ChassisSpeeds> driveSpeeds = driveStateTable.getStructTopic("Speeds", ChassisSpeeds.struct).publish();
    private final StructArrayPublisher<SwerveModuleState> driveModuleStates = driveStateTable.getStructArrayTopic("ModuleStates", SwerveModuleState.struct).publish();
    private final StructArrayPublisher<SwerveModuleState> driveModuleTargets = driveStateTable.getStructArrayTopic("ModuleTargets", SwerveModuleState.struct).publish();
    private final StructArrayPublisher<SwerveModulePosition> driveModulePositions = driveStateTable.getStructArrayTopic("ModulePositions", SwerveModulePosition.struct).publish();
    private final DoublePublisher driveTimestamp = driveStateTable.getDoubleTopic("Timestamp").publish();
    private final DoublePublisher driveOdometryFrequency = driveStateTable.getDoubleTopic("OdometryFrequency").publish();

    /* Robot pose for field positioning */
    private final NetworkTable poses = tableInstance.getTable("Pose");
    private DoubleArrayPublisher fieldPub = poses.getDoubleArrayTopic("robotPose").publish();
    private final StringPublisher fieldTypePub = poses.getStringTopic(".type").publish();

    /* Mechanisms to represent the swerve module states */
    private final Mechanism2d[] m_moduleMechanisms = new Mechanism2d[] 
    {
        new Mechanism2d(1, 1),
        new Mechanism2d(1, 1),
        new Mechanism2d(1, 1),
        new Mechanism2d(1, 1),
    };
    /* A direction and length changing ligament for speed representation */
    private final MechanismLigament2d[] m_moduleSpeeds = new MechanismLigament2d[] 
    {
        m_moduleMechanisms[0].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
        m_moduleMechanisms[1].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
        m_moduleMechanisms[2].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
        m_moduleMechanisms[3].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
    };
    /* A direction changing and length constant ligament for module direction */
    private final MechanismLigament2d[] m_moduleDirections = new MechanismLigament2d[] 
    {
        m_moduleMechanisms[0].getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
        m_moduleMechanisms[1].getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
        m_moduleMechanisms[2].getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
        m_moduleMechanisms[3].getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
    };

    private static final double[] m_poseArray = new double[3];

    private Telemetry ()
    {
        //turretYawRaw.setPersistent(true);
    }

    public void update ()
    {
        mostRecentAim.set(Robot.instance.robotContainer.mostRecentAim ? "Pass" : "Shoot");
        hoodOffset.set(Robot.instance.robotContainer.pitchOffset);

        Command intakeCommand = Intake.getInstance().getCurrentCommand();
        this.intakeCommand.set(intakeCommand == null ? "" : intakeCommand.getName());
        intakeMainVelocity.set(Intake.getInstance().getVelocity().in(RotationsPerSecond));
        intakeMainVoltage.set(Intake.getInstance().getVoltage().in(Volts));
        
        Command intakeExtensionCommand = IntakeExtension.getInstance().getCurrentCommand();
        this.intakeExtensionCommand.set(intakeExtensionCommand == null ? "" : intakeExtensionCommand.getName());
        intakeExtensionPosition.set(IntakeExtension.getInstance().getPosition().in(Rotations));
        intakeExtensionVelocity.set(IntakeExtension.getInstance().getVelocity().in(RotationsPerSecond));
        intakeExtensionVoltage.set(IntakeExtension.getInstance().getVoltage().in(Volts));

        /*
        Command turretCommand = Turret.getInstance().getCurrentCommand();
        this.turretCommand.set(turretCommand == null ? "" : turretCommand.getName());
        turretPosition.set(Turret.getInstance().getPosition().in(Degrees));
        turretTargetPosition.set(Turret.getInstance().getDesiredPosition().in(Degrees));
        turretVelocity.set(Turret.getInstance().getVelocity().in(DegreesPerSecond));
        turretVoltage.set(Turret.getInstance().getVoltage().in(Volts));
        turretReadyToShoot.set(Turret.getInstance().readyToShoot());
        */
        
        Command hoodCommand = Hood.getInstance().getCurrentCommand();
        this.hoodCommand.set(hoodCommand == null ? "" : hoodCommand.getName());
        hoodPosition.set(Hood.getInstance().getPosition().in(Degrees));
        hoodTargetPosition.set(Hood.getInstance().getDesiredPosition().in(Degrees));
        hoodVelocity.set(Hood.getInstance().getVelocity().in(DegreesPerSecond));
        hoodVoltage.set(Hood.getInstance().getVoltage().in(Volts));
        hoodReadyToShoot.set(Hood.getInstance().readyToShoot());

        Command shooterCommand = Shooter.getInstance().getCurrentCommand();
        this.shooterCommand.set(shooterCommand == null ? "" : shooterCommand.getName());
        shooterLeftVelocity.set(Shooter.getInstance().getLeftVelocity().in(RotationsPerSecond));
        shooterLeftVoltage.set(Shooter.getInstance().getLeftVoltage().in(Volts));
        shooterRightVelocity.set(Shooter.getInstance().getRightVelocity().in(RotationsPerSecond));
        shooterRightVoltage.set(Shooter.getInstance().getRightVoltage().in(Volts));
        
        Command hopperCommand = Hopper.getInstance().getCurrentCommand();
        this.hopperCommand.set(hopperCommand == null ? "" : hopperCommand.getName());
        hopperPosition.set(Hopper.getInstance().getPosition().in(Rotations));
        hopperVelocity.set(Hopper.getInstance().getVelocity().in(RotationsPerSecond));
        hopperVoltage.set(Hopper.getInstance().getVoltage().in(Volts));
        // hopperTarget.set(Hopper.getInstance().getDesiredPosition().in(Rotations));

        Command climbCommand = Climb.getInstance().getCurrentCommand();
        this.climbCommand.set(climbCommand == null ? "" : climbCommand.getName());
        climbElevatorVelocity.set(Climb.getInstance().getElevatorVelocity().in(RotationsPerSecond));
        climbElevatorVoltage.set(Climb.getInstance().getElevatorVoltage().in(Volts));
        climbElevatorPosition.set(Climb.getInstance().getElevatorPosition().in(Rotations));
        climbElevatorTarget.set(Climb.getInstance().getElevatorTargetPosition().in(Rotations));
        climbClimbVoltage.set(Climb.getInstance().getClimbVoltage().in(Volts));


        //turretYawRawPublisher.set(Turret.getInstance().getPosition().in(Rotations));

        fuels.set(SimulationState.getInstance().fuelPositionsRaw);

        fuelsInRobot.set(SimulationState.getInstance().fuelsInRobot);
        fuelsInBlueHub.set(SimulationState.getInstance().fuelsInBlueHub);
        fuelsInRedHub.set(SimulationState.getInstance().fuelsInRedHub);
        fuelsInBlueOutpost.set(SimulationState.getInstance().fuelsInBlueOutpost);
        fuelsInRedOutpost.set(SimulationState.getInstance().fuelsInRedOutpost);

        Command indexerCommand = Indexer.getInstance().getCurrentCommand();
        this.indexerCommand.set(indexerCommand == null ? "" : indexerCommand.getName());
        indexerVelocity.set(Indexer.getInstance().getVelocity().in(RotationsPerSecond));
        indexerVoltage.set(Indexer.getInstance().getVoltage().in(Volts));

        Command shooterIndexerCommand = ShooterIndexer.getInstance().getCurrentCommand();
        this.shooterIndexerCommand.set(shooterIndexerCommand == null ? "" : shooterIndexerCommand.getName());
        shooterIndexerVelocity.set(ShooterIndexer.getInstance().getVelocity().in(RotationsPerSecond));
        shooterIndexerVoltage.set(ShooterIndexer.getInstance().getVoltage().in(Volts));


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
    /** Accept the swerve drive state and telemeterize it to SmartDashboard and SignalLogger. */
    public void telemeterize(SwerveDriveState state) 
    {
        /* Telemeterize the swerve drive state */
        drivePose.set(state.Pose);
        driveSpeeds.set(state.Speeds);
        driveModuleStates.set(state.ModuleStates);
        driveModuleTargets.set(state.ModuleTargets);
        driveModulePositions.set(state.ModulePositions);
        driveTimestamp.set(state.Timestamp);
        driveOdometryFrequency.set(1.0 / state.OdometryPeriod);

        /* Also write to log file */
        SignalLogger.writeStruct("DriveState/Pose", Pose2d.struct, state.Pose);
        SignalLogger.writeStruct("DriveState/Speeds", ChassisSpeeds.struct, state.Speeds);
        SignalLogger.writeStructArray("DriveState/ModuleStates", SwerveModuleState.struct, state.ModuleStates);
        SignalLogger.writeStructArray("DriveState/ModuleTargets", SwerveModuleState.struct, state.ModuleTargets);
        SignalLogger.writeStructArray("DriveState/ModulePositions", SwerveModulePosition.struct, state.ModulePositions);
        SignalLogger.writeDouble("DriveState/OdometryPeriod", state.OdometryPeriod, "seconds");

        /* Telemeterize the pose to a Field2d */
        fieldTypePub.set("Field2d");

        m_poseArray[0] = state.Pose.getX();
        m_poseArray[1] = state.Pose.getY();
        m_poseArray[2] = state.Pose.getRotation().getDegrees();
        fieldPub.set(m_poseArray);

        /* Telemeterize each module state to a Mechanism2d */
        for (int i = 0; i < 4; ++i) 
        {
            m_moduleSpeeds[i].setAngle(state.ModuleStates[i].angle);
            m_moduleDirections[i].setAngle(state.ModuleStates[i].angle);
            m_moduleSpeeds[i].setLength(state.ModuleStates[i].speedMetersPerSecond / (2 * Robot.instance.robotContainer.MaxSpeed));
        }
    }


    public static Telemetry getInstance ()
    {
        if (instance == null) instance = new Telemetry();
        return instance;
    }
}
