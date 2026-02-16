package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.function.Function;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;

import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import frc.robot.simulation.BallConstants;

public class Constants 
{
    public static final double MAX_VOLTAGE = 12.0;
    
    public static final double EPSILON = 0.1;

    public static final double ROBOT_DIAMETER = 0.889;

    public static final Pose2d ZEROING_POSE = new Pose2d(4.028649 - ROBOT_DIAMETER / 2.0, 4.034536, new Rotation2d(0.0));

    public static final double TRANSLATION_SLOW_MULTIPLIER = 0.2;
    public static final double ROTATION_SLOW_MULTIPLIER = 0.4;

    public class Vision {
        public static final String kCamera1Name = "limelight";
        public static final Transform3d kRobotToCam1 = new Transform3d(
                new Translation3d(Units.inchesToMeters(5.472), Units.inchesToMeters(-10.5), Units.inchesToMeters(7.482)),
                new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(26.1), 0)); // TODO bc this is copied from 2025 code

        public static final double linTagStdDevs = 0.1;
        public static final double angTagStdDevs = 999999;
        public static final Matrix<N3, N1> kTagStdDevs = VecBuilder.fill(0.1, 0.1, 99999);
        public static final Matrix<N3, N1> stateStdDevs = VecBuilder.fill(0.05, 0.05, 0.1);
    }

    public class Drivetrain 
    {
        public static final double MAX_VELOCITY = 1.0;
        public static final Translation2d HUB = new Translation2d(0, 0); //TODO 
    }

    public class Shooter
    {
        public static final int LEFT_MASTER_ID = 19; // TODO
        public static final int LEFT_FOLLOWER_ID = 20; // TODO
        public static final int RIGHT_MASTER_ID = 21; // TODO
        public static final int RIGHT_FOLLOWER_ID = 22; // TODO
        
        public static final double STATOR_CURRENT_LIMIT = 90.0;
        public static final double SUPPLY_CURRENT_LIMIT = 90.0;

        public static final double MM_CRUISE_VELOCITY = 60.0;
		public static final double MM_ACCELERATION = 60.0;
		public static final double MM_JERK = 240.0;
		public static final InvertedValue INVERTED = InvertedValue.CounterClockwise_Positive;
        
        public static final double GEAR_RATIO = 1.0; // TODO

		public static final double KP = 0.042562; // TODO
		public static final double KI = 0.0; // TODO
		public static final double KD = 0.0; // TODO
		
        public static final double KS = 0.0022734; // TODO
		public static final double KG = 0.0; // TODO
        public static final double KV = 0.11931; // TODO
        public static final double KA = 0.0059936; // TODO

        public static final double DEFAULT_VELOCITY = 0.1; // TODO rotations per second (nonzero to decrease startup time)
        public static final double SHOOT_VELOCITY = 10.0; // TODO rotations per second 

        public static final double INCREASE_VELOCITY = 0.5; // TODO
        public static final double MAX_VELOCITY = 20.0; // TODO
        
        public static final ChassisReference MECHANICAL_ORIENTATION = ChassisReference.CounterClockwise_Positive;
    }

    public class Intake
    {
        public static int ID = 23;

        public static final double STATOR_CURRENT_LIMIT = 90.0;
        public static final double SUPPLY_CURRENT_LIMIT = 90.0;

        public static final double INTAKE_VOLTAGE = 5.8; //TODO
        public static final double DEFAULT_INTAKE_VOLTAGE = 0.0; //TODO
        public static final double EJECT_VOLTAGE = -2.5; //TODO

        public static final double KP = 0.0023821; // TODO
		public static final double KI = 0.0; // TODO
		public static final double KD = 0.0; // TODO
		
        public static final double KS = 0.0017035; // TODO
        public static final double KV = 0.12364; // TODO
        public static final double KA = 0.0078492; // TODO
        public static final InvertedValue INVERTED = InvertedValue.Clockwise_Positive;

        
        public static final double GEAR_RATIO = 8.0/9.0; //TODO
        
        public static final ChassisReference MECHANICAL_ORIENTATION = ChassisReference.CounterClockwise_Positive;
    }

    public class IntakeExtension
    {
        public static int ID = 24;

        public static final double STATOR_CURRENT_LIMIT = 90.0;
        public static final double SUPPLY_CURRENT_LIMIT = 90.0;

        public static final double EXTENDING_VOLTAGE = 2.5;
        public static final double RETRACTING_VOLTAGE = -2.5;

        public static final double KP = 0.0023821; // TODO
		public static final double KI = 0.0; // TODO
		public static final double KD = 0.0; // TODO
		
        public static final double KS = 0.0017035; // TODO
        public static final double KV = 0.12364; // TODO
        public static final double KA = 0.0078492; // TODO
        public static final InvertedValue INVERTED = InvertedValue.CounterClockwise_Positive;

        
        public static final double GEAR_RATIO = 6.25; //TODO
        
        public static final ChassisReference MECHANICAL_ORIENTATION = ChassisReference.CounterClockwise_Positive;

        public static final double STALLING_CURRENT = 44;

        public static final double MIN_HEIGHT = 0.0; // TODO
        public static final double MAX_HEIGHT = 1.0; // TODO
    }

    public class Hopper
    {
        public static final int ID = 25;
        
        public static final double STATOR_CURRENT_LIMIT = 90.0;
        public static final double SUPPLY_CURRENT_LIMIT = 90.0;

        /*
        public static final double MM_CRUISE_VELOCITY = 60.0;
		public static final double MM_ACCELERATION = 60.0;
		public static final double MM_JERK = 240.0;
        */
		public static final InvertedValue INVERTED = InvertedValue.CounterClockwise_Positive;
        
        public static final double GEAR_RATIO = 1.0; // TODO

		public static final double KP = 0.24194; // TODO
		public static final double KI = 0.0; // TODO
		public static final double KD = 0.0022479; // TODO
		
        public static final double KS = 0.0041693; // TODO
		public static final double KG = 0.054566; // TODO
        public static final double KV = 0.019146; // TODO
        public static final double KA = 0.0018688; // TODO

        public static final double MIN_POSITION = 0.1; // TODO min hopper position (rotations) for simulation
        public static final double MAX_POSITION = 10.0; // TODO max hopper position (rotations) for simulation
 
        public static final double FORWARD_VOLTAGE = 1.0; // TODO
        public static final double BACKWARD_VOLTAGE = -1.0; // TODO

        public static final double STALLING_CURRENT = 50;

        public static final ChassisReference MECHANICAL_ORIENTATION = ChassisReference.CounterClockwise_Positive;
    }
    
    public class Hood
    {
        public static final int MASTER_ID = 26;
        public static final int FOLLOWER_ID = 27;
        
        public static final double STATOR_CURRENT_LIMIT = 90.0;
        public static final double SUPPLY_CURRENT_LIMIT = 90.0;

        public static final double MM_CRUISE_VELOCITY = 1.0; // TODO
		public static final double MM_ACCELERATION = 4.0;  // TODO
		public static final double MM_JERK = 8.0; // TODO
		public static final InvertedValue INVERTED = InvertedValue.CounterClockwise_Positive;
        
        public static final double GEAR_RATIO = 1.0; // TODO

		public static final double KP = 1.0; // TODO
		public static final double KI = 0.9; // TODO
		public static final double KD = 0.038151; // TODO
		
        public static final double KS = 0.0024809; // TODO
        public static final double KV = 0.1205; // TODO
        public static final double KA = 0.0042197; // TODO
		public static final double KG = 0.017908; // TODO

        public static final double FORWARD_SOFTWARE_LIMIT_THRESHOLD = 4.82; // TODO
		public static final double REVERSE_SOFTWARE_LIMIT_THRESHOLD = -0.01; // TODO
        
        public static final ChassisReference MECHANICAL_ORIENTATION = ChassisReference.CounterClockwise_Positive;

        public static final double MOMENT_OF_INERTIA = 0.001; // TODO (kg m^2)

        public static final double LENGTH = 0.5; // TODO meters
        public static final double MIN_ANGLE = 5.0; // TODO degrees
        public static final double MAX_ANGLE = 70.0; // TODO degrees
    
        public static final double STALLING_CURRENT = 50.0;

        public static final double ZEROING_VOLTAGE = -1.0; // TODO
    }

    /*
    public class Turret // this can be ignored for the present
    {
        public static final double ERROR_THRESHOLD = 1.0; // TODO degrees

        public static final int MOTOR_ID = 28; // TODO

        public static final double STATOR_CURRENT_LIMIT = 90.0;
        public static final double SUPPLY_CURRENT_LIMIT = 90.0;

        public static final double MM_CRUISE_VELOCITY = 60.0;
		public static final double MM_ACCELERATION = 60.0;
		public static final double MM_JERK = 240.0;
		public static final InvertedValue INVERTED = InvertedValue.CounterClockwise_Positive;
        
        public static final double GEAR_RATIO = 1.0; // TODO

		public static final double KP = 1.0; // TODO
		public static final double KI = 0.1; // TODO
		public static final double KD = 0.0; // TODO
		
        public static final double KV = 0.0; // TODO
		public static final double KG = 0.0; // TODO

        public static final double FORWARD_SOFTWARE_LIMIT_THRESHOLD = 4.82; // TODO
		public static final double REVERSE_SOFTWARE_LIMIT_THRESHOLD = -0.01; // TODO
        
        public static final ChassisReference MECHANICAL_ORIENTATION = ChassisReference.CounterClockwise_Positive;
    }
        */

    public class Simulation
    {
        public static final double FIELD_HEIGHT = 8.069326;
        public static final double FIELD_WIDTH = 16.540988;

        // excluding steel barrier
        public static final Rectangle2d DEPOT = new Rectangle2d(
            new Translation2d(0.0, FIELD_HEIGHT - 1.570736 - 0.0762),
            new Translation2d(0.6858 - 0.0762, FIELD_HEIGHT - 1.570736 - 1.0668 + 0.0762));
        
        public static final Translation2d FIELD_CENTER = new Translation2d(FIELD_WIDTH / 2.0, FIELD_HEIGHT / 2.0);

        public static final Function<Double, Double> ROTATE_X = (Double x) -> x + 2 * (FIELD_CENTER.getX() - x);
        public static final Function<Double, Double> ROTATE_Y = (Double y) -> y + 2 * (FIELD_CENTER.getY() - y);

        public static final Translation2d CENTER_UPPER_REFERENCE = new Translation2d(FIELD_CENTER.getX(), FIELD_CENTER.getY() + 0.0254);
        public static final Translation2d CENTER_LOWER_REFERENCE = new Translation2d(FIELD_CENTER.getX(), FIELD_CENTER.getY() - 0.0254);

        public static final double FUEL_DIAMETER = 0.15;
        public static final double MIN_FUEL_MASS = 0.203;
        public static final double MAX_FUEL_MASS = 0.227;

        public static final int TOTAL_FUEL = 504;
        public static final int FUELS_TAKEN_BY_OTHER_ROBOTS = 0;

        public static final Rectangle2d HUB_CONTENTS = new Rectangle2d(new Pose2d(new Translation2d(4.574794, 4.059936), new Rotation2d()), 1.1938, 1.1938);

        public static final Rectangle2d OUTPOST = new Rectangle2d(
            new Translation2d(-0.84 - 0.5 * FUEL_DIAMETER, 0.331 - 0.5 * FUEL_DIAMETER), 
            new Translation2d(-0.0708 + 0.5 * FUEL_DIAMETER, 1.008 + 0.5 * FUEL_DIAMETER));

        public static final Translation3d OUTPOST_SPAWN_LOCATION_LOWER = new Translation3d(0.5 * FUEL_DIAMETER, 0.2655, 0.714);
        public static final Translation3d OUTPOST_SPAWN_LOCATION_UPPER = new Translation3d(0.5 * FUEL_DIAMETER, 1.0735, 0.714);

        public static final BallConstants BALL_CONSTANTS = new BallConstants(
            (MAX_FUEL_MASS + MIN_FUEL_MASS) / 2.0,
            FUEL_DIAMETER / 2.0, 1.2, 0.30, 1.2, 0.35, 9.81, 20);

        public static final double HUB_INTAKE_HEIGHT = 1.8288;
    }
    public static final class Climb 
    {
        public static final int ELEVATOR_ID = 29;
        public static final int HINGE_ID = 30;

        public static final InvertedValue ELEVATOR_INVERTED = InvertedValue.CounterClockwise_Positive; // TODO
        public static final InvertedValue CLIMB_INVERTED = InvertedValue.CounterClockwise_Positive; // TODO

        public static final ChassisReference ELEVATOR_MECHANICAL_ORIENTATION = ChassisReference.CounterClockwise_Positive;
        public static final ChassisReference CLIMB_MECHANICAL_ORIENTATION = ChassisReference.CounterClockwise_Positive;

        public static final double ELEVATOR_GEAR_RATIO = 1.0;
        public static final double CLIMB_GEAR_RATIO = 1.0;

        public static final double KP_ELEVATOR = 1.0; // TODO
        public static final double KP_CLIMB = 1.0; // TODO
        public static final double KI_ELEVATOR = 0.0; //TODO
        public static final double KI_CLIMB = 0.0; // TODO
        public static final double KD_ELEVATOR = 0.0; //TODO
        public static final double KD_CLIMB = 0.037935; // TODO

        public static final Current STATOR_CURRENT_LIMIT = Amps.of(100 + 40); // TODO
        public static final Current ELEVATOR_STALLING_CURRENT = Amps.of(100);
        public static final Current SUPPLY_CURRENT_LIMIT = Amps.of(100 + 20); // TODO

        public static final Angle CLIMB_POSITION_LEVEL_1 = Rotations.of(2.0);  // rotations // TODO
        public static final Angle CLIMB_POSITION_LEVEL_2 = Rotations.of(4.0);  // rotations // TODO
        public static final Angle CLIMB_POSITION_LEVEL_3 = Rotations.of(6.0);  // rotations // TODO
		
        public static final double KS = 0.0025457; // TODO
        public static final double KV = 0.018935; // TODO
        public static final double KA = 0.0016851; // TODO
		public static final double KG = 0.016529; // TODO
        public static final Angle MAX_ERROR = Rotations.of(0.1);

        public static final Voltage ELEVATOR_GO_DOWN_VOLTAGE = Volts.of(-2.0); //TODO
        public static final Voltage CLIMB_DEPLOY_VOLTAGE = Volts.of(1.0); // TODO
        public static final Voltage CLIMB_UNDEPLOY_VOLTAGE = Volts.of(-1.0); // TODO

        public static final double ELEVATOR_MIN_HEIGHT = 0.0;
        public static final double ELEVATOR_MAX_HEIGHT = 10.0;

        public static final double MM_CRUISE_VELOCITY = 6.0;
		public static final double MM_ACCELERATION = 5.0;
		public static final double MM_JERK = 24.0;
    }

    public class Indexer {

        public static final int ID = 31; //TODO

        public static final InvertedValue INVERTED = InvertedValue.Clockwise_Positive; // TODO
        public static final double STATOR_CURRENT_LIMIT = 80.0;
        public static final double SUPPLY_CURRENT_LIMIT = 80.0;

        public static final double KS = 0.0048438; //TODO
        public static final double KV = 0.12361; //TODO
        public static final double KA = 0.0089587; //TODO
        public static final double KG = 0.0; //TODO
        
        public static final double KP = 0.093687; //TODO
        public static final double KI = 0.0; //TODO
        public static final double KD = 0.0; //TODO

        
        /*
        public static final double MM_CRUISE_VELOCITY = 0.0;  
        public static final double MM_ACCELERATION = 0.0;
        public static final double MM_JERK = 0.0;
        */

        public static final double MAX_VELOCITY = 1.0; // TODO rotations per second
        public static final double DEFAULT_VELOCITY = 0.0; // TODO rotations per second

        public static final double GEAR_RATIO = 1.0; // TODO

        public static final ChassisReference MECHANICAL_ORIENTATION = ChassisReference.Clockwise_Positive;
    }

    public class ShooterIndexer
    {
        public static final int ID = 32; // TODO

        public static final InvertedValue INVERTED = InvertedValue.CounterClockwise_Positive;
        public static final double STATOR_CURRENT_LIMIT = 80.0;
        public static final double SUPPLY_CURRENT_LIMIT = 80.0;

        public static final double DEFAULT_VOLTAGE = 0.0;
        public static final double INTAKE_VOLTAGE = 1.0; // TODO
        
        public static final double KS = 0.0048438; // TODO
        public static final double KV = 0.12361; // TODO
        public static final double KA = 0.0089587; // TODO
        public static final double KG = 0.0; // TODO
        
        public static final double KP = 0.093687; // TODO
        public static final double KI = 0.0; // TODO
        public static final double KD = 0.0; // TODO

        public static final double GEAR_RATIO = 1.0; // TODO

        public static final ChassisReference MECHANICAL_ORIENTATION = ChassisReference.CounterClockwise_Positive;
    }

    // Generated by the 2026 Tuner X Swerve Project Generator
    // https://v6.docs.ctr-electronics.com/en/stable/docs/tuner/tuner-swerve/index.html
    public class TunerConstants {
        // Both sets of gains need to be tuned to your individual robot.

        // The steer motor uses any SwerveModule.SteerRequestType control request with
        // the
        // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
        public static final Slot0Configs steerGains = new Slot0Configs()
                .withKP(20.823).withKI(0).withKD(0.49617)
                .withKS(0.0/*0.27579*/).withKV(2.4256).withKA(0.055487)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
        // When using closed-loop control, the drive motor uses the control
        // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
        public static final Slot0Configs driveGains = new Slot0Configs()
                .withKP(0.1).withKI(0).withKD(0)
                .withKS(0).withKV(0.124);

        // The closed-loop output type to use for the steer motors;
        // This affects the PID/FF gains for the steer motors
        public static final ClosedLoopOutputType kSteerClosedLoopOutput = ClosedLoopOutputType.Voltage;
        // The closed-loop output type to use for the drive motors;
        // This affects the PID/FF gains for the drive motors
        public static final ClosedLoopOutputType kDriveClosedLoopOutput = ClosedLoopOutputType.Voltage;

        // The type of motor used for the drive motor
        public static final DriveMotorArrangement kDriveMotorType = DriveMotorArrangement.TalonFX_Integrated;
        // The type of motor used for the drive motor
        public static final SteerMotorArrangement kSteerMotorType = SteerMotorArrangement.TalonFX_Integrated;

        // The remote sensor feedback type to use for the steer motors;
        // When not Pro-licensed, Fused*/Sync* automatically fall back to Remote*
        public static final SteerFeedbackType kSteerFeedbackType = SteerFeedbackType.FusedCANcoder;

        // The stator current at which the wheels start to slip;
        // This needs to be tuned to your individual robot
        public static final Current kSlipCurrent = Amps.of(120);

        // Initial configs for the drive and steer motors and the azimuth encoder; these
        // cannot be null.
        // Some configs will be overwritten; check the `with*InitialConfigs()` API
        // documentation.
        public static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration();
        public static final TalonFXConfiguration steerInitialConfigs = new TalonFXConfiguration()
                .withCurrentLimits(
                        new CurrentLimitsConfigs()
                                // Swerve azimuth does not require much torque output, so we can set a
                                // relatively low
                                // stator current limit to help avoid brownouts without impacting performance.
                                .withStatorCurrentLimit(Amps.of(60))
                                .withStatorCurrentLimitEnable(true));
        public static final CANcoderConfiguration encoderInitialConfigs = new CANcoderConfiguration();
        // Configs for the Pigeon 2; leave this null to skip applying Pigeon 2 configs
        public static final Pigeon2Configuration pigeonConfigs = null;

        // CAN bus that the devices are located on;
        // All swerve devices must share the same CAN bus
        public static final CANBus kCANBus = new CANBus("", "./logs/example.hoot");

        // Theoretical free speed (m/s) at 12 V applied output;
        // This needs to be tuned to your individual robot
        public static final LinearVelocity kSpeedAt12Volts = MetersPerSecond.of(4.39);

        // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
        // This may need to be tuned to your individual robot
        public static final double kCoupleRatio = 4.5;

        public static final double kDriveGearRatio = 7.03125;
        public static final double kSteerGearRatio = 26.09090909090909;
        public static final Distance kWheelRadius = Inches.of(2);

        public static final boolean kInvertLeftSide = false;
        public static final boolean kInvertRightSide = true;

        public static final int kPigeonId = 2;

        // These are only used for simulation
        public static final MomentOfInertia kSteerInertia = KilogramSquareMeters.of(0.01);
        public static final MomentOfInertia kDriveInertia = KilogramSquareMeters.of(0.01);
        // Simulated voltage necessary to overcome friction
        public static final Voltage kSteerFrictionVoltage = Volts.of(0.2);
        public static final Voltage kDriveFrictionVoltage = Volts.of(0.2);

        public static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
                .withCANBusName(kCANBus.getName())
                .withPigeon2Id(kPigeonId)
                .withPigeon2Configs(pigeonConfigs);

        public static final SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> ConstantCreator = new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
                .withDriveMotorGearRatio(kDriveGearRatio)
                .withSteerMotorGearRatio(kSteerGearRatio)
                .withCouplingGearRatio(kCoupleRatio)
                .withWheelRadius(kWheelRadius)
                .withSteerMotorGains(steerGains)
                .withDriveMotorGains(driveGains)
                .withSteerMotorClosedLoopOutput(kSteerClosedLoopOutput)
                .withDriveMotorClosedLoopOutput(kDriveClosedLoopOutput)
                .withSlipCurrent(kSlipCurrent)
                .withSpeedAt12Volts(kSpeedAt12Volts)
                .withDriveMotorType(kDriveMotorType)
                .withSteerMotorType(kSteerMotorType)
                .withFeedbackSource(kSteerFeedbackType)
                .withDriveMotorInitialConfigs(driveInitialConfigs)
                .withSteerMotorInitialConfigs(steerInitialConfigs)
                .withEncoderInitialConfigs(encoderInitialConfigs)
                .withSteerInertia(kSteerInertia)
                .withDriveInertia(kDriveInertia)
                .withSteerFrictionVoltage(kSteerFrictionVoltage)
                .withDriveFrictionVoltage(kDriveFrictionVoltage);

        // Front Left
        public static final int kFrontLeftDriveMotorId = 12;
        public static final int kFrontLeftSteerMotorId = 11;
        public static final int kFrontLeftEncoderId = 17;
        public static final Angle kFrontLeftEncoderOffset = Rotations.of(-0.4521484375);
        public static final boolean kFrontLeftSteerMotorInverted = false;
        public static final boolean kFrontLeftEncoderInverted = false;

        public static final Distance kFrontLeftXPos = Inches.of(11.375);
        public static final Distance kFrontLeftYPos = Inches.of(10.375);

        // Front Right
        public static final int kFrontRightDriveMotorId = 10;
        public static final int kFrontRightSteerMotorId = 9;
        public static final int kFrontRightEncoderId = 18;
        public static final Angle kFrontRightEncoderOffset = Rotations.of(0.25927734375);
        public static final boolean kFrontRightSteerMotorInverted = false;
        public static final boolean kFrontRightEncoderInverted = false;

        public static final Distance kFrontRightXPos = Inches.of(11.375);
        public static final Distance kFrontRightYPos = Inches.of(-10.375);

        // Back Left
        public static final int kBackLeftDriveMotorId = 4;
        public static final int kBackLeftSteerMotorId = 3;
        public static final int kBackLeftEncoderId = 14;
        public static final Angle kBackLeftEncoderOffset = Rotations.of(0.184326171875);
        public static final boolean kBackLeftSteerMotorInverted = false;
        public static final boolean kBackLeftEncoderInverted = false;

        public static final Distance kBackLeftXPos = Inches.of(-11.375);
        public static final Distance kBackLeftYPos = Inches.of(10.375);

        // Back Right
        public static final int kBackRightDriveMotorId = 8;
        public static final int kBackRightSteerMotorId = 7;
        public static final int kBackRightEncoderId = 16;
        public static final Angle kBackRightEncoderOffset = Rotations.of(-0.014404296875);
        public static final boolean kBackRightSteerMotorInverted = false;
        public static final boolean kBackRightEncoderInverted = false;

        public static final Distance kBackRightXPos = Inches.of(-11.375);
        public static final Distance kBackRightYPos = Inches.of(-10.375);

    }

    public static final Translation3d HUB_TARGET_POSITION = new Translation3d(Simulation.HUB_CONTENTS.getCenter().getTranslation().getX(),
        Simulation.HUB_CONTENTS.getCenter().getTranslation().getX(),
        Simulation.HUB_INTAKE_HEIGHT);

    public static final double HOOD_BASE_HEIGHT = 0.10; // TODO

    public static final Translation3d PASS_LEFT_TARGET_POSITION = new Translation3d(2.010664, 2.010664, 1.0);
    public static final Translation3d PASS_RIGHT_TARGET_POSITION = new Translation3d(2.010664, Simulation.FIELD_HEIGHT - 2.010664, 1.0);

    public static final Angle HARDCODE_HOOD_PITCH = Degrees.of(20.0);
    public static final double HARDCODE_VELOCITY = 5.0;

    public static final double PITCH_OFFSET_UNIT = 5.0; // TODO
    public static final double FLYWHEEL_OFFSET_UNIT = 2.5; // TODO

}