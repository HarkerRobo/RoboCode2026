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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.math.geometry.Translation2d;

import com.pathplanner.lib.util.FlippingUtil;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
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
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterIndexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeExtension;
import frc.robot.subsystems.Climb;

/**
 * Central telemtry publisher for subsystems and drivetrain state
 * Publishes robot data to NetWorkTables
 */
public class Telemetry 
{
    private static Telemetry instance;

    private NetworkTableInstance tableInstance = NetworkTableInstance.getDefault();
    private NetworkTable table = tableInstance.getTable("1072");

    private StringPublisher mostRecentAim = table.getStringTopic("most recent aim").publish();
    private DoublePublisher hoodOffset = table.getDoubleTopic("hood offset").publish();
    private DoublePublisher flywheelOffset = table.getDoubleTopic("flywheel offset").publish();
    private BooleanPublisher intakeTriggered = table.getBooleanTopic("intake triggered").publish();
    private BooleanPublisher intakeExtended = table.getBooleanTopic("intake extended").publish();
    public BooleanPublisher aligned = table.getBooleanTopic("aligned").publish();
    public DoubleArrayPublisher currents = table.getDoubleArrayTopic("currents").publish();

    private NetworkTable intake = table.getSubTable("Intake");
    private StringPublisher intakeCommand = intake.getStringTopic("main command").publish();
    private DoublePublisher intakeTargetVelocity = intake.getDoubleTopic("target velocity (rot per s)").publish();
    private DoublePublisher intakeLeftVelocity = intake.getDoubleTopic("left velocity (rot per s)").publish();
    private DoublePublisher intakeLeftVoltage = intake.getDoubleTopic("left voltage (V)").publish();
    private DoublePublisher intakeLeftCurrent = intake.getDoubleTopic("left current (A) ").publish();
    private DoublePublisher intakeRightVelocity = intake.getDoubleTopic("right velocity (rot per s)").publish();
    private DoublePublisher intakeRightVoltage = intake.getDoubleTopic("right voltage (V)").publish();
    private DoublePublisher intakeRightCurrent = intake.getDoubleTopic("right current (A) ").publish();

    private NetworkTable intakeExtension = table.getSubTable("Intake Extension");
    private StringPublisher intakeExtensionCommand = intakeExtension.getStringTopic("command").publish();
    private DoublePublisher intakeExtensionVelocity = intakeExtension.getDoubleTopic("velocity (rot per s)").publish();
    private DoublePublisher intakeExtensionVoltage = intakeExtension.getDoubleTopic("voltage (V)").publish();
    private DoublePublisher intakeExtensionCurrent = intakeExtension.getDoubleTopic("current (A)").publish();

    private NetworkTable hood = table.getSubTable("Hood");
    private StringPublisher hoodCommand = hood.getStringTopic("command").publish();
    private DoublePublisher hoodPosition = hood.getDoubleTopic("position (°)").publish();
    private DoublePublisher hoodTargetPosition = hood.getDoubleTopic("target position (°)").publish();
    private DoublePublisher hoodVoltage = hood.getDoubleTopic("voltage (V)").publish();
    private BooleanPublisher hoodReadyToShoot = hood.getBooleanTopic("ready to shoot?").publish();
    private DoublePublisher hoodStatorCurrent = hood.getDoubleTopic("stator current (A)").publish();
    
    private NetworkTable shooter = table.getSubTable("Shooter");
    private StringPublisher shooterCommand = shooter.getStringTopic("command").publish();
    private DoublePublisher shooterVelocity = shooter.getDoubleTopic("velocity (rot per s)").publish();
    private DoublePublisher shooterEffectiveVelocity = shooter.getDoubleTopic("effective velocity (m per s)").publish();
    private DoublePublisher shooterTargetVelocity = shooter.getDoubleTopic("target velocity (rot per s)").publish();
    private DoublePublisher shooterTargetEffectiveVelocity = shooter.getDoubleTopic("target effective velocity (m per s)").publish();
    private DoublePublisher shooterVoltage = shooter.getDoubleTopic("voltage (V)").publish();
    private BooleanPublisher shooterReadyToShoot = shooter.getBooleanTopic("ready to shoot?").publish();

    private NetworkTable climb = table.getSubTable("Climb");
    private StringPublisher climbCommand = climb.getStringTopic("command").publish();
    private DoublePublisher climbClimbWheelsVelocity = climb.getDoubleTopic("climb wheels velocity (rot per s)").publish();
    private DoublePublisher climbClimbWheelsVoltage = climb.getDoubleTopic("climb wheels voltage (V)").publish();
    private DoublePublisher climbClimbWheelsPosition = climb.getDoubleTopic("climb wheels position (rot)").publish();
    private DoublePublisher climbSpoolingVoltage = climb.getDoubleTopic("spooling voltage (V)").publish();
    private DoublePublisher climbSpoolingPosition = climb.getDoubleTopic("spooling position (rot)").publish();

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
    public DoublePublisher test1 = simulation.getDoubleTopic("Current heading").publish();
    public DoublePublisher test2 = simulation.getDoubleTopic("Desired heading").publish();

    private NetworkTable indexer = table.getSubTable("Indexer");
    private StringPublisher indexerCommand = indexer.getStringTopic("command").publish();
    private DoublePublisher indexerMainVelocity = indexer.getDoubleTopic("main velocity (rps)").publish();
    private DoublePublisher indexerMainVoltage = indexer.getDoubleTopic("main voltage (V)").publish();
    
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
    private final DoublePublisher distanceToShoot = driveStateTable.getDoubleTopic("DistanceToShoot").publish();
    private final DoublePublisher driveHeading = driveStateTable.getDoubleTopic("Drivetrain Heading (Degrees)").publish();

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


    private final NetworkTable inputs = tableInstance.getTable("Control Inputs");
    private final DoubleSubscriber hoodAngle = inputs.getDoubleTopic("Hood Angle (degrees)").subscribe(75.0);
    private final DoubleSubscriber shooterSpeed = inputs.getDoubleTopic("Shooter Speed (degrees)").subscribe(10.0);

    /**
     * Constructs telemetry
     * Private because this class is a singleton
     */
    private Telemetry ()
    {
        hoodAngle.getTopic().publish().set(75.0);
        shooterSpeed.getTopic().publish().set(10.0);
        //turretYawRaw.setPersistent(true);
    }

    public void update ()
    {
        mostRecentAim.set(Robot.instance.robotContainer.mostRecentAim ? "Pass" : "Shoot");
        hoodOffset.set(Robot.instance.robotContainer.pitchOffset);
        flywheelOffset.set(Robot.instance.robotContainer.flywheelOffset);
        intakeTriggered.set(Robot.instance.robotContainer.intakeTriggered);
        intakeExtended.set(Robot.instance.robotContainer.intakeExtended);
        // currents.set(Robot.instance.robotContainer.powerDistributionTracker.getAllCurrents());

        Command intakeCommand = Intake.getInstance().getCurrentCommand();
        this.intakeCommand.set(intakeCommand == null ? "" : intakeCommand.getName());
        intakeTargetVelocity.set(Intake.getInstance().getTargetVelocity().in(RotationsPerSecond));
        intakeLeftVelocity.set(Intake.getInstance().getLeftVelocity().in(RotationsPerSecond));
        intakeLeftVoltage.set(Intake.getInstance().getLeftVoltage().in(Volts));
        intakeLeftCurrent.set(Intake.getInstance().getLeftStatorCurrent().in(Amps));
        intakeRightVelocity.set(Intake.getInstance().getRightVelocity().in(RotationsPerSecond));
        intakeRightVoltage.set(Intake.getInstance().getRightVoltage().in(Volts));
        intakeRightCurrent.set(Intake.getInstance().getRightStatorCurrent().in(Amps));
        
        Command intakeExtensionCommand = IntakeExtension.getInstance().getCurrentCommand();
        this.intakeExtensionCommand.set(intakeExtensionCommand == null ? "" : intakeExtensionCommand.getName());
        intakeExtensionVelocity.set(IntakeExtension.getInstance().getVelocity().in(RotationsPerSecond));
        intakeExtensionVoltage.set(IntakeExtension.getInstance().getVoltage().in(Volts));
        intakeExtensionCurrent.set(IntakeExtension.getInstance().getStatorCurrent().in(Amps));

        Command hoodCommand = Hood.getInstance().getCurrentCommand();
        this.hoodCommand.set(hoodCommand == null ? "" : hoodCommand.getName());
        hoodPosition.set(Hood.getInstance().getPosition().in(Degrees));
        hoodTargetPosition.set(Hood.getInstance().getDesiredPosition().in(Degrees));
        hoodVoltage.set(Hood.getInstance().getVoltage().in(Volts));
        hoodReadyToShoot.set(Hood.getInstance().readyToShoot());
        hoodStatorCurrent.set(Hood.getInstance().getStatorCurrent());

        Command shooterCommand = Shooter.getInstance().getCurrentCommand();
        this.shooterCommand.set(shooterCommand == null ? "" : shooterCommand.getName());
        shooterVelocity.set(Shooter.getInstance().getVelocity().in(RotationsPerSecond));
        shooterEffectiveVelocity.set(Shooter.getInstance().getEffectiveVelocity().in(MetersPerSecond));
        shooterTargetVelocity.set(Shooter.getInstance().getTargetVelocity().in(RotationsPerSecond));
        shooterTargetEffectiveVelocity.set(Shooter.getInstance().getTargetEffectiveVelocity().in(MetersPerSecond));
        shooterVoltage.set(Shooter.getInstance().getVoltage().in(Volts));
        shooterReadyToShoot.set(Shooter.getInstance().readyToShoot());
        
        Command climbCommand = Climb.getInstance().getCurrentCommand();
        this.climbCommand.set(climbCommand == null ? "" : climbCommand.getName());
        climbClimbWheelsVelocity.set(Climb.getInstance().getClimbWheelsVelocity().in(RotationsPerSecond));
        climbClimbWheelsVoltage.set(Climb.getInstance().getClimbWheelsVoltage().in(Volts));
        climbClimbWheelsPosition.set(Climb.getInstance().getClimbWheelsPosition().in(Rotations));
        climbSpoolingVoltage.set(Climb.getInstance().getSpoolingVoltage().in(Volts));
        climbSpoolingPosition.set(Climb.getInstance().getSpoolingPosition().in(Rotations));


        //turretYawRawPublisher.set(Turret.getInstance().getPosition().in(Rotations));

        fuels.set(SimulationState.getInstance().fuelPositionsRaw);

        fuelsInRobot.set(SimulationState.getInstance().fuelsInRobot);
        fuelsInBlueHub.set(SimulationState.getInstance().fuelsInBlueHub);
        fuelsInRedHub.set(SimulationState.getInstance().fuelsInRedHub);
        fuelsInBlueOutpost.set(SimulationState.getInstance().fuelsInBlueOutpost);
        fuelsInRedOutpost.set(SimulationState.getInstance().fuelsInRedOutpost);

        Command indexerCommand = Indexer.getInstance().getCurrentCommand();
        this.indexerCommand.set(indexerCommand == null ? "" : indexerCommand.getName());
        indexerMainVelocity.set(Indexer.getInstance().getMainVelocity().in(RotationsPerSecond));
        indexerMainVoltage.set(Indexer.getInstance().getMainVoltage().in(Volts));

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
        /*
        double posX = Robot.instance.robotContainer.drivetrain.getState().Pose.getX();
        double posY = Robot.instance.robotContainer.drivetrain.getState().Pose.getY();
        double rot = Robot.instance.robotContainer.drivetrain.getState().Pose.getRotation().getRadians();
        double endX = posX + Constants.ROBOT_DIAMETER*Math.sqrt(2)/2*Math.cos(rot+Math.PI/4);
        double endY = posY + Constants.ROBOT_DIAMETER*Math.sqrt(2)/2*Math.sin(rot+Math.PI/4);
        double startX = posX + Constants.ROBOT_DIAMETER*Math.sqrt(2)/2*Math.cos(rot-Math.PI/4);
        double startY = posY + Constants.ROBOT_DIAMETER*Math.sqrt(2)/2*Math.sin(rot-Math.PI/4);

        double midPointX = posX + Constants.ROBOT_DIAMETER/2*Math.cos(rot);
        double midPointY = posY + Constants.ROBOT_DIAMETER/2*Math.sin(rot);

        test.set(new Translation3d[] 
        {
            new Translation3d(-0.84, 0.331, 0.075),
            new Translation3d(-0.0708, 1.008, 0.075),
            new Translation3d(posX, posY, 0),
            new Translation3d(endX, endY, 0),
            new Translation3d(startX, startY, 0),
            new Translation3d(midPointX, midPointY, 0)
        }
            */;
        test.set(new Translation3d[]
        {
            //Util.Robot.instance.robotContainer.drivetrain.getState().Pose.getTranslation()
        }
     
        );
    }
    /** 
     * Accept the swerve drive state and telemeterize it to SmartDashboard and SignalLogger. 
     * @param state Current drivetrain state
     */
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
        driveHeading.set(state.Pose.getRotation().getDegrees());
        Translation2d targetPose = (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) ? 
                    Constants.HUB_TARGET_POSITION.toTranslation2d() : 
                    FlippingUtil.flipFieldPosition(Constants.HUB_TARGET_POSITION.toTranslation2d());
        distanceToShoot.set(state.Pose.getTranslation().getDistance(targetPose));

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

    /**
     * Get hood angle
     */
    public double getHoodAngle()
    {
        return hoodAngle.get();
    }
    
    /**
     * Get shooter speed
     */
    public double getShooterSpeed()
    {
        return shooterSpeed.get();
    }

    /**
     * Returns the singleton telemetry instance
     * @return  Telemetry singleton
     */
    public static Telemetry getInstance ()
    {
        if (instance == null) instance = new Telemetry();
        return instance;
    }
}
