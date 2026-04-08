// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Function;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.*;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.TunerConstants;
import frc.robot.RobotContainer.PassDirection;
import frc.robot.commands.climb.ClimbDown;
import frc.robot.commands.climb.ClimbUp;
import frc.robot.commands.climb.Unspool;
import frc.robot.commands.climb.Spool;
import frc.robot.commands.drive.DriveToPose;
import frc.robot.commands.drive.RotateToAngle;
import frc.robot.commands.hood.AimToAngle;
import frc.robot.commands.hood.AimToAngleInPerpetuity;
import frc.robot.commands.hood.HoodManual;
import frc.robot.commands.hood.ZeroHood;
import frc.robot.commands.indexer.IndexerStartDefaultSpeed;
import frc.robot.commands.indexer.IndexerStartEjectSpeed;
import frc.robot.commands.indexer.IndexerStartFullSpeed;
import frc.robot.commands.intake.AgitateIntake;
import frc.robot.commands.intake.StartDefaultIntake;
import frc.robot.commands.intake.StartEjectIntake;
import frc.robot.commands.intake.StartRunIntake;
import frc.robot.commands.intakeextension.ExtendIntake;
import frc.robot.commands.intakeextension.RetractIntake;
import frc.robot.commands.shooter.ShooterTargetSpeed;
import frc.robot.commands.shooter.ShooterTargetSpeedInPerpetuity;
import frc.robot.commands.shooterindexer.ShooterIndexerStartDefaultSpeed;
import frc.robot.commands.shooterindexer.ShooterIndexerStartEjectSpeed;
import frc.robot.commands.shooterindexer.ShooterIndexerStartFullSpeed;
import frc.robot.subsystems.*;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.Modules;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeExtension;
import frc.robot.util.Util;



public class RobotContainer 
{
    public enum AlignDirection
    {
        Right,
        Left
    }
    
    public double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private AlignDirection alignDirection = AlignDirection.Left;
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    
    public final CommandXboxController driver = new CommandXboxController(0);
    public final CommandXboxController operator = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = Modules.createDrivetrain();

    public SendableChooser<Command> testCommandChooser = new SendableChooser<>();
    public ArrayList<SendableChooser<SubsystemStatus>> modeChoosers = new ArrayList<>();

    Function<RobotContainer.AlignDirection, Command> setDirectionFactory = ((AlignDirection direction) ->
        
        new Command() {
            public void execute () {System.out.println(direction); Robot.instance.robotContainer.setAlignDirection(direction);}
            public boolean isFinished () {return true;}});

    Command alignLeft = setDirectionFactory.apply(AlignDirection.Left);
    Command alignRight = setDirectionFactory.apply(AlignDirection.Right);

    private SendableChooser<Command> autonChooser;
    public static int INTAKE_INDEX = 0;
    public static int INTAKE_EXTENSION_INDEX = 1;
    public static int CLIMB_INDEX = 2;
    public static int HOOD_INDEX = 3;
    public static int HOPPER_INDEX = 4;
    public static int INDEXER_INDEX = 5;
    public static int SHOOTER_INDEX = 6;
    public static int SHOOTER_INDEXER_INDEX = 7;
    public static int INDEXES = 8;

    public enum SubsystemStatus {Enabled, Simulated, Disabled};

    private boolean isSlow = false;
    public boolean mostRecentAim = false; // false = shoot; true = pass
    public boolean intakeTriggered = false; // true if intake has been enabled
    public boolean intakeExtended = true; //true if intake and hopper have been extended // TODO reverse
  
    private List<Command> commands = new ArrayList<>(40);
  
    public static enum PassDirection {Left, Right, Automatic};

    private PassDirection direction = PassDirection.Automatic; // Default
        
    private Supplier<Command> stow;
    private Command shoot;
    private Command midPass;
    private Command hardPass;
    private Command hardShoot;
    private Command revShoot;
    private Command revPass;

    private SlewRateLimiter accelerationLimiter = new SlewRateLimiter(Constants.ACCELERATION_LIMIT);
    public PowerDistribution powerDistributionTracker = new PowerDistribution();

    /**
     * Sets the desired pass direction mode
     * @param newDirection  New pass direction mode
     */
    public void setPassDirection(PassDirection newDirection) {
        direction = newDirection;
    }

    public PassDirection getPassDirection() {
        return direction;
    }

    /**
     * Determines if the robot treats itself as on the left side
     * @return  True if the robot treats itself as on the left side; false otherwise
     */
    public boolean onLeftSide()
    {
      if (direction == PassDirection.Left) return true;
      if (direction == PassDirection.Right) return false;
      return (Util.onLeftSide(drivetrain));
    }

    public SubsystemStatus getStatus(int subsystem)
    {
        return modeChoosers.get(subsystem).getSelected();
    }

    private static String subsystemName(int subsystem)
    {
        return switch(subsystem) {
            case 0 -> "Intake";
            case 1 -> "IntakeExtension";
            case 2 -> "Climb";
            case 3 -> "Hood";
            case 4 -> "Hopper";
            case 5 -> "Indexer";
            case 6 -> "Shooter";
            case 7 -> "ShooterIndexer";
            default -> "";
        };
    }
  
        
    /**
     * Constructs the RobotContainer 
     * and initializes subsystem mode choosers
     */    
    public RobotContainer() 
    {
        for (int i = 0; i < INDEXES; i++)
        {
            SendableChooser<SubsystemStatus> chooser = new SendableChooser<>();

            if (Robot.isSimulation())
            {
                chooser.setDefaultOption("Sim", SubsystemStatus.Simulated);
                chooser.addOption("Off", SubsystemStatus.Disabled);
            }
            else
            {
                chooser.setDefaultOption("On", SubsystemStatus.Enabled);
                chooser.addOption("Off", SubsystemStatus.Disabled);
                chooser.addOption("Sim", SubsystemStatus.Simulated);
            }
            modeChoosers.add(chooser);
            SmartDashboard.putData(subsystemName(i), chooser);
        }
    }

    /**
     * Post-construction initialization
     * Builds commands, registers PathPlanner NamedCommands, configures default subsystem commands, 
     * selects control binding layout and publishes choosers to SmartDashBoard
     */
    public void init()
    {
        stow = ()->
            new IndexerStartDefaultSpeed()
            .andThen(new StartDefaultIntake())
            .andThen(Shooter.getInstance().runOnce(()->Shooter.getInstance().setVoltage(Volts.of(0.0))))
            .andThen(new ShooterIndexerStartDefaultSpeed())
            .andThen(new AimToAngle(75.0))
            .withName("Stow"); // must stay a supplier
        
        shoot = 
            new AimToAngle(()->Util.calculateShootPitch(drivetrain).in(Degrees))
            .andThen(new ShooterTargetSpeed(()->Util.calculateShootVelocity(drivetrain)))
            .andThen(new WaitUntilCommand(() -> Shooter.getInstance().readyToShoot() && Hood.getInstance().readyToShoot()))
            .andThen(new IndexerStartFullSpeed())
            .andThen(new ShooterIndexerStartFullSpeed())
            .withName("Shoot");
        
        midPass = 
        // lucas wanted to remove the auto-aligning (4/3/26, at contra costa) 
        // new RotateToAngle(drivetrain,
        //     () -> onLeftSide() ? Constants.PASS_LEFT_TARGET_POSITION.toTranslation2d()
        //                        : Constants.PASS_RIGHT_TARGET_POSITION.toTranslation2d(), true)
                new AimToAngle(Constants.MID_PASS_ANGLE)
                .andThen(new ShooterTargetSpeed(Constants.MID_PASS_VELOCITY))
                .andThen(new WaitUntilCommand(() -> Shooter.getInstance().readyToShoot() && Hood.getInstance().readyToShoot()))
                .andThen(new IndexerStartFullSpeed())
                .andThen(new ShooterIndexerStartFullSpeed())
            .withName("MidPass");
        
        hardPass = 
        // lucas wanted to remove the auto-aligning (4/3/26, at contra costa) 
        // new RotateToAngle(drivetrain,
        //     () -> onLeftSide() ? Constants.PASS_LEFT_TARGET_POSITION.toTranslation2d()
        //                        : Constants.PASS_RIGHT_TARGET_POSITION.toTranslation2d(), true)
                new AimToAngle(Constants.HARD_PASS_ANGLE)
                .andThen(new ShooterTargetSpeed(Constants.HARD_PASS_VELOCITY))
                .andThen(new WaitUntilCommand(() -> Shooter.getInstance().readyToShoot() && Hood.getInstance().readyToShoot()))
                .andThen(new IndexerStartFullSpeed())
                .andThen(new ShooterIndexerStartFullSpeed())
            .withName("HardPass");
        

        revShoot = Commands.none()
            .andThen(new ShooterIndexerStartDefaultSpeed())
            .andThen(new ShooterTargetSpeed(()->Util.calculateShootVelocity(drivetrain)))
            .andThen(new WaitUntilCommand(()->Shooter.getInstance().readyToShoot()))
            //.andThen(
            //     Commands.runOnce(()->driver.setRumble(RumbleType.kBothRumble, 1.0)))
            .withName("RevShoot");


        testCommandChooser.setDefaultOption("None", Commands.none());
        testCommandChooser.addOption("Climb/ClimbUp", new ClimbUp());
        testCommandChooser.addOption("Climb/ClimbDown", new ClimbDown());
        testCommandChooser.addOption("Climb/SpoolUntilStall", new Spool());
        testCommandChooser.addOption("Climb/Unspool", new Unspool());
        testCommandChooser.addOption("Hood/AimToAngle[75°]", new AimToAngle(75.0));
        testCommandChooser.addOption("Hood/AimToAngle[60°]", new AimToAngle(60.0));
        testCommandChooser.addOption("Hood/AimToAngle[70°]", new AimToAngle(70.0));
        testCommandChooser.addOption("Hood/ZeroHood", new ZeroHood());
        testCommandChooser.addOption("Indexer/IndexerStartDefaultSpeed", new IndexerStartDefaultSpeed());
        testCommandChooser.addOption("Indexer/IndexerStartFullSpeed", new IndexerStartFullSpeed());
        testCommandChooser.addOption("Intake/StartDefaultIntake", new StartDefaultIntake());
        testCommandChooser.addOption("Intake/StartRunIntake", new StartRunIntake());
        testCommandChooser.addOption("Intake/StartEjectIntake", new StartEjectIntake());
        testCommandChooser.addOption("Intake/AgitateIntake", new AgitateIntake());
        testCommandChooser.addOption("IntakeExtension/ExtendIntake", new ExtendIntake());
        testCommandChooser.addOption("IntakeExtension/RetractIntake", new RetractIntake());
        testCommandChooser.addOption("Shooter/ShooterTargetSpeed[10]", new ShooterTargetSpeed(10.0));
        testCommandChooser.addOption("Shooter/ShooterTargetSpeed[" + Constants.HARDCODE_VELOCITY + "]", new ShooterTargetSpeed(Constants.HARDCODE_VELOCITY));
        testCommandChooser.addOption("Shooter/ShooterTargetSpeed[" + 0 + "]", Shooter.getInstance().runOnce(()->Shooter.getInstance().setVoltage(Volts.of(0.0))));
        testCommandChooser.addOption("ShooterIndexer/ShooterIndexerStartDefaultSpeed", new ShooterIndexerStartDefaultSpeed());
        testCommandChooser.addOption("ShooterIndexer/ShooterIndexerStartFullSpeed", new ShooterIndexerStartFullSpeed());
        testCommandChooser.addOption("ZeroDrivetrain", 
                drivetrain.runOnce(() -> {
                if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red)
                {
                    drivetrain.seedFieldCentric(Rotation2d.k180deg);
                }
                else
                {
                    drivetrain.seedFieldCentric();
                }
            })
                .andThen(drivetrain.runOnce(() -> drivetrain.resetPose(
                            (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) ? 
                            FlippingUtil.flipFieldPose(Constants.ZEROING_POSE) : Constants.ZEROING_POSE)))
                .withName("ZeroDrivetrain"));

        SmartDashboard.putData("Test a Command", testCommandChooser);
        SmartDashboard.putData(CommandScheduler.getInstance());

        NamedCommands.registerCommand("ExtendIntake", Commands.runEnd(
            ()->IntakeExtension.getInstance().setVoltage(Volts.of(Constants.IntakeExtension.EXTENDING_VOLTAGE)),
            ()->IntakeExtension.getInstance().setVoltage(Volts.of(Constants.IntakeExtension.HOLDING_EXTEND_VOLTAGE))));
        NamedCommands.registerCommand("RetractIntake", new RetractIntake().alongWith(new StartRunIntake())
            .andThen(new StartDefaultIntake()));
        NamedCommands.registerCommand("StartRunIntake", new StartRunIntake());
        NamedCommands.registerCommand("StartDefaultIntake", new StartDefaultIntake());
        NamedCommands.registerCommand("EjectIntake (with timeout)", 
            new StartEjectIntake()
            .andThen(new WaitCommand(1.0))
            .andThen(new StartDefaultIntake()));
        NamedCommands.registerCommand("HardShoot", hardShoot);
        NamedCommands.registerCommand("RevShoot",
            new ShooterTargetSpeed(()->Util.calculateShootVelocity(drivetrain)));
        NamedCommands.registerCommand("LinearAgitateIntake",
            new StartRunIntake()
            .andThen(new RetractIntake().withTimeout(2.0))
            .andThen(new ExtendIntake().withTimeout(1.0))
            .andThen(new RetractIntake().withTimeout(2.0))
            .andThen(new ExtendIntake().withTimeout(1.0))
            .andThen(new RetractIntake().withTimeout(2.0)));

        NamedCommands.registerCommand("Shoot", 
            new AimToAngle(()->Util.calculateShootPitch(drivetrain).in(Degrees))
            .andThen(new ShooterTargetSpeed(()->Util.calculateShootVelocity(drivetrain)))
            .andThen(new WaitUntilCommand(()->Shooter.getInstance().readyToShoot() && Hood.getInstance().readyToShoot()))
            .andThen(new ShooterIndexerStartFullSpeed())
            .andThen(new IndexerStartFullSpeed()) // load to shoot
            .andThen(Commands.run(()->{}))
        .withName("Shoot"));

        NamedCommands.registerCommand("Shoot2",
            new AimToAngle(Constants.AUTO_SHOOT_2_ANGLE)
            .andThen(new ShooterTargetSpeed(Constants.AUTO_SHOOT_2_VELOCITY))
            .andThen(new WaitUntilCommand(()->Shooter.getInstance().readyToShoot() && Hood.getInstance().readyToShoot()))
            .andThen(new ShooterIndexerStartFullSpeed())
            .andThen(new IndexerStartFullSpeed()) // load to shoot
            .andThen(Commands.run(()->{})));

        NamedCommands.registerCommand("Shoot1",
            new AimToAngle(Constants.AUTO_SHOOT_1_ANGLE)
            .andThen(new ShooterTargetSpeed(Constants.AUTO_SHOOT_1_VELOCITY))
            .andThen(new WaitUntilCommand(()->Shooter.getInstance().readyToShoot() && Hood.getInstance().readyToShoot()))
            .andThen(new ShooterIndexerStartFullSpeed())
            .andThen(new IndexerStartFullSpeed()) // load to shoot
            .andThen(Commands.run(()->{})));

        NamedCommands.registerCommand("Stow", stow.get());
        
        autonChooser = AutoBuilder.buildAutoChooser();
        
        SmartDashboard.putData("Auton Chooser", autonChooser);

        // ----------------------- DEFAULT BINDINGS HERE -------------------------

        Hood.getInstance().setDefaultCommand(new HoodManual());

        // -------------------- CHANGE BINDING SETTINGS HERE ---------------------
        
        boolean useDebuggingBindings = false; // mainly for sysid or debugging
        
        if (useDebuggingBindings) configureDebugBindings();
        else
        {
            configureDriverBindings();
            configureOperatorBindings();
        }
        drivetrain.registerTelemetry(Telemetry.getInstance()::telemeterize);
    }


    /**
     * Configures debugging bindings
     * Used during development, not for match play
     */
    private void configureDebugBindings()
    {
        // driver.a().whileTrue(Shooter.getInstance().sysIdQuasistatic(Direction.kForward));
        // driver.b().whileTrue(Shooter.getInstance().sysIdQuasistatic(Direction.kReverse));
        // driver.x().whileTrue(Shooter.getInstance().sysIdDynamic(Direction.kForward));
        // driver.y().whileTrue(Shooter.getInstance().sysIdDynamic(Direction.kReverse));

        // driver.x().whileTrue(new Spool());
        // driver.y().whileTrue(new ClimbUp());
        // driver.a().whileTrue(new ClimbDown());
        // driver.b().whileTrue(new Unspool());

        //driver.button(1).onTrue(alignLeft.andThen(new DriveToPose(drivetrain)));
        //driver.button(2).onTrue(alignRight.andThen(new DriveToPose(drivetrain)));

        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> 
                        drive.withVelocityX(-driver.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                        .withVelocityY(-driver.getLeftX() * MaxSpeed * (isSlow ? Constants.TRANSLATION_SLOW_MULTIPLIER : 1.0)) // Drive left with negative X (left)
                        .withRotationalRate(-driver.getRightX() * MaxAngularRate * (isSlow ? Constants.ROTATION_SLOW_MULTIPLIER : 1.0)) // Drive counterclockwise with negative X (left)
                    ).withName("SwerveManual"));}
    

    /**
     * Configures default driver binding
     * Allow basic driving even when main bindings are unavailable
     */
    private void configureDriverBindings() 
    {
        driver.a().whileTrue(new RotateToAngle(drivetrain, () -> AlignConstants.HUB, false)
            .withName("ShootAlign"));
        
        // driver.y().onTrue(
        //     new ShooterTargetSpeed(Constants.HARDCODE_VELOCITY)
        //     .andThen(new AimToAngle(Constants.HARDCODE_HOOD_PITCH.in(Degrees)))
        //     .andThen(new ShooterIndexerStartFullSpeed())
        //     .andThen(new IndexerStartFullSpeed())
        //     .withName("HardShoot"));

        // driver.y().onFalse(stow.get());

        driver.y().onTrue(hardPass);
        driver.y().onFalse(stow.get());
        
        driver.b().onTrue(midPass);
        driver.b().onFalse(stow.get());
        
        driver.x().onTrue(
            new ShooterTargetSpeed(Constants.HARDCODE_VELOCITY_2)
            .andThen(new AimToAngle(Constants.HARDCODE_HOOD_PITCH_2.in(Degrees)))
            .andThen(new ShooterIndexerStartFullSpeed())
            .andThen(new IndexerStartFullSpeed())
            .withName("HardShoot2"));

        driver.x().onFalse(stow.get());

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> 
                        drive.withVelocityX(/*accelerationLimiter.calculate(*/-driver.getLeftY() * MaxSpeed * (isSlow ? Constants.TRANSLATION_SLOW_MULTIPLIER : 1.0)/*)*/) // Drive forward with negative Y (forward)
                        .withVelocityY(/*accelerationLimiter.calculate(*/-driver.getLeftX() * MaxSpeed * (isSlow ? Constants.TRANSLATION_SLOW_MULTIPLIER : 1.0)/*)*/) // Drive left with negative X (left)
                        .withRotationalRate(-driver.getRightX() * MaxAngularRate * (isSlow ? Constants.ROTATION_SLOW_MULTIPLIER : 1.0)) // Drive counterclockwise with negative X (left)
                    ).withName("SwerveManual"));

        // tested
        driver.leftTrigger().whileTrue(new StartEndCommand(()->isSlow = true, ()->isSlow = false).withName("ToggleSlow"));

        driver.rightTrigger().onTrue(
            shoot
            // new AimToAngle(()->Telemetry.getInstance().getHoodAngle())
            // .andThen(new ShooterTargetSpeed(()->Telemetry.getInstance().getShooterSpeed()))
            // .andThen(new WaitCommand(2.0))
            // .andThen(new ShooterIndexerStartFullSpeed())
            // .andThen(new IndexerStartFullSpeed())
            );

        driver.rightTrigger().onFalse(stow.get());

        driver.leftBumper().onTrue(stow.get().andThen(Commands.print("Stowing")).withName("Stow"));

        //changed from retract/extand hopper and intake
        driver.rightBumper().onTrue(
            Commands.runOnce(()->{
                if (intakeTriggered)
                {
                    intakeTriggered = false;
                    CommandScheduler.getInstance().schedule(
                        new StartDefaultIntake()
                        .withName("DeactivateIntake"));
                }
                else if (true)
                {
                    intakeTriggered = true;
                    CommandScheduler.getInstance().schedule(
                        new StartRunIntake()
                        .withName("ActivateIntake"));
                }
        }));
        
        
        driver.button(8) // menu button/right paddle
            .onTrue(revShoot);

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
                drivetrain.applyRequest(() -> idle).ignoringDisable(true).withName("Drivetrain Set Idle"));
    }

    public void configureOperatorBindings()
    {
        operator.start().onTrue(
                drivetrain.runOnce(() -> {
                if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red)
                {
                    drivetrain.seedFieldCentric(Rotation2d.k180deg);
                }
                else
                {
                    drivetrain.seedFieldCentric();
                }
            })
                .andThen(drivetrain.runOnce(() -> drivetrain.resetPose(
                            (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) ? 
                            FlippingUtil.flipFieldPose(Constants.ZEROING_POSE) : Constants.ZEROING_POSE)))
                .withName("ZeroDrivetrain"));

        operator.back().whileTrue(new ZeroHood()
            .alongWith(new Unspool())
            .withName("ZeroHood+Climb"));

        operator.leftTrigger().onTrue(new StartEjectIntake()
            .andThen(new IndexerStartEjectSpeed())
            .andThen(new ShooterIndexerStartEjectSpeed()));
        operator.leftTrigger().onFalse(new StartRunIntake()
            .andThen(new IndexerStartDefaultSpeed())
            .andThen(new ShooterIndexerStartDefaultSpeed()));

        operator.rightTrigger().onTrue(
            new ShooterTargetSpeed(Constants.Shooter.SOFT_PASS_VELOCITY)
            .andThen(new AimToAngle(60.0))
            .andThen(new IndexerStartFullSpeed())
            .andThen(new ShooterIndexerStartFullSpeed())
            .withName("SoftPass"));
        operator.rightTrigger().onFalse(stow.get());

        // operator.leftBumper().onTrue(Commands.runOnce(()->
        // {
        //     if (direction == PassDirection.Right) direction = PassDirection.Automatic;
        //     else direction = PassDirection.Right;
        // }));
        // operator.rightBumper().onTrue(Commands.runOnce(()->
        // {
        //     if (direction == PassDirection.Left) direction = PassDirection.Automatic;
        //     else direction = PassDirection.Left;
        // }));

        operator.x().onTrue(Intake.getInstance().runOnce(()->Intake.getInstance().setVelocity(
                RotationsPerSecond.of(Constants.Intake.REDUCED_INTAKE_VELOCITY)))
            .andThen(Commands.runOnce(()->intakeTriggered = true))
            .andThen(new RetractIntake())
            .andThen(new StartDefaultIntake())
            .andThen(Commands.runOnce(()->
            {
                intakeExtended = false;
            }
            )).withName("RetractIntake"));
        operator.b().onTrue(new StartDefaultIntake()
            .andThen(Commands.runOnce(()->intakeTriggered = false))
            .andThen(new ExtendIntake())
            .andThen(Commands.runOnce(()->
            {
                intakeExtended = true;
            }
            )).withName("ExtendIntake"));
    }

    public AlignDirection getAlignDirection ()
    {
        return alignDirection;
    }

    public void setAlignDirection (AlignDirection direction)
    {
        this.alignDirection = direction;
    } 

    public Command getAutonomousCommand() 
    {
        return autonChooser.getSelected();
    }
}
