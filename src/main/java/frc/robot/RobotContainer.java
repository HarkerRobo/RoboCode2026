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
import frc.robot.commands.hood.HoodManualDown;
import frc.robot.commands.hood.HoodManualUp;
import frc.robot.commands.hood.ZeroHood;
import frc.robot.commands.indexer.IndexerDefaultSpeed;
import frc.robot.commands.indexer.IndexerFullSpeed;
import frc.robot.commands.intake.AgitateIntake;
import frc.robot.commands.intake.DefaultIntake;
import frc.robot.commands.intake.EjectIntake;
import frc.robot.commands.intake.ExtendIntake;
import frc.robot.commands.intake.RetractIntake;
import frc.robot.commands.intake.RunIntake;
import frc.robot.commands.shooter.ShooterDefaultSpeed;
import frc.robot.commands.shooter.ShooterTargetSpeed;
import frc.robot.commands.shooterindexer.ShooterIndexerDefaultSpeed;
import frc.robot.commands.shooterindexer.ShooterIndexerFullSpeed;
import frc.robot.subsystems.*;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.Modules;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeExtension;
import frc.robot.util.IndependentCommand;
import frc.robot.util.PerpetuallyIndependentCommand;
import frc.robot.util.Util;



public class RobotContainer 
{
    public enum AlignDirection
    {
        Center,
        Right,
        Left
    }
    
    /**
     * This is necessary because shoot, pass, revshoot, revpass, and hardshoot all call shooter
     * independently, meaning that the shooter subsystem is not added as a requirement, nor can it
     * be manually added or else the calling of the independentcommand would schedule a new command
     * which cancels the original command; since we actually want to prevent conflict between shoot,
     * pass, revshoot, revpass, and hardshoot, and because this is quite important (or else revpass
     * will not cancel revshoot) a possibility would be to manually cancel all of the other commands
     * when a single command in the list is running, but there is no method that I am aware of for
     * determining which of the commands was scheduled most recently, meaning that the resolution
     * of conflicts could devolve into an arbitrary choice which causes more problems (such as
     * preventing revPass from being scheduled if revShoot has already been scheduled, or vice
     * versa, depending on how the code is implemented). Instead, we create a subsystem mimic which
     * persuades the command scheduler to cancel all commands that are a commandgroup including
     * a command on "shooterCommandFakeSubsystem" which does nothing but nicely adds a common 
     * requirement to the same instance of the same subsystem, so that the most recent command 
     * "requiring" the shooterCommandFakeSubsystem "subsystem" cancels all other running ones in a 
     * consistent and expectable manner.
     */
    private Subsystem shooterCommandFakeSubsystem = new SubsystemBase() {};

    public double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private AlignDirection alignDirection = AlignDirection.Center;
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
    Command alignCenter = setDirectionFactory.apply(AlignDirection.Center);
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
    public double pitchOffset = 0.0;
    public double leftFlywheelOffset = 0.0;
    public double rightFlywheelOffset = 0.0;
  
    private List<Command> commands = new ArrayList<>(40);
  
    public static enum PassDirection {Left, Right, Automatic};

    private PassDirection direction = PassDirection.Automatic; // Default
        
    private Supplier<Command> stow;
    private Command shoot;
    private Command pass;
    private Command hardShoot;
    private Command revShoot;
    private Command revPass;

    private SlewRateLimiter accelerationLimiter = new SlewRateLimiter(Constants.ACCELERATION_LIMIT);
    public PowerDistribution powerDistributionTracker = new PowerDistribution();

    public void setPassDirection(PassDirection newDirection) {
        direction = newDirection;
    }

    public PassDirection getPassDirection() {
        return direction;
    }

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

    public void init()
    {
        // tested in sim
        stow = ()->Commands.runOnce(()->CommandScheduler.getInstance().cancel(commands.toArray(new Command[0])))
            .andThen(new AimToAngle(75.0));
        // because a command instance cannot be scheduled to independent triggers
        

        // tested in sim
        shoot = new RotateToAngle(drivetrain, ()->AlignConstants.HUB)
            //Commands.none()
            .alongWith(new IndependentCommand(track(new AimToAngle(()->Util.calculateShootPitch(drivetrain).in(Degrees)))))
            .alongWith(new IndependentCommand(track(new ShooterTargetSpeed(()->Util.calculateShootVelocity(drivetrain)))))
            // .alongWith(new AimToAngle(75.0))
            // .alongWith(new IndependentCommand(new ShooterTargetSpeed(()->8.0 + leftFlywheelOffset, ()->8.0 + rightFlywheelOffset)))
            .andThen(new WaitUntilCommand(
                ()->Shooter.getInstance().readyToShoot() // we will see if this works lol
                 && 
                 Hood.getInstance().readyToShoot()
                 ))
            .andThen(new IndependentCommand(track(new IndexerFullSpeed())))
            .andThen(track(new ShooterIndexerFullSpeed())) // load to shoot
            .finallyDo(()->{
                CommandScheduler.getInstance().schedule(stow.get());
            })
            .andThen(shooterCommandFakeSubsystem.runOnce(()->{}))
            .withName("Shoot");
        
        // tested in sim
        pass = new RotateToAngle(drivetrain,
            () -> onLeftSide() ? Constants.PASS_LEFT_TARGET_POSITION.toTranslation2d()
                               : Constants.PASS_RIGHT_TARGET_POSITION.toTranslation2d())
            //Commands.none()
            .alongWith(new AimToAngle(() -> Util.calculatePassPitch(drivetrain).in(Degrees) + pitchOffset))
            .andThen(new IndependentCommand(track(new ShooterTargetSpeed(
                ()->Util.calculatePassVelocity(drivetrain) + leftFlywheelOffset,
                ()->Util.calculatePassVelocity(drivetrain) + rightFlywheelOffset))))
            .andThen(new WaitUntilCommand(
                    () -> Shooter.getInstance().readyToShoot()
                            && Hood.getInstance().readyToShoot()))
            .andThen(new ShooterIndexerFullSpeed()) // load to pass
            .andThen(shooterCommandFakeSubsystem.runOnce(()->{}))
            .finallyDo(() -> {
                CommandScheduler.getInstance().schedule(stow.get());
            })
            .withName("Pass");
        

        // tested in sim
        hardShoot = //new AimToAngle(Constants.HARDCODE_HOOD_PITCH.in(Degrees))
            Commands.none()
            .alongWith(new IndependentCommand(track(new ShooterTargetSpeed(()->Constants.HARDCODE_VELOCITY))))
            .andThen(new IndependentCommand(track(new RunIntake())))
            .andThen(new IndependentCommand(track(new IndexerFullSpeed())))
            .andThen(new WaitUntilCommand(()->Shooter.getInstance().readyToShoot()))
            .andThen(new ShooterIndexerFullSpeed())
            .finallyDo(()->CommandScheduler.getInstance().schedule(new ShooterDefaultSpeed()))
            .andThen(shooterCommandFakeSubsystem.runOnce(()->{}))
            .withName("HardShoot");

        revShoot = 
                Commands.runOnce(()->mostRecentAim = false)
            //.andThen(new IndependentCommand(track(new ShooterIndexerDefaultSpeed())))
            .andThen(
                new IndependentCommand(track(new ShooterTargetSpeed(()->Util.calculateShootVelocity(drivetrain)))))
                // new IndependentCommand(new ShooterTargetSpeed(()->8.0 + leftFlywheelOffset, ()->8.0 + rightFlywheelOffset)))
            .andThen(new WaitUntilCommand(()->Shooter.getInstance().readyToShoot()))
            //.andThen(
            //     Commands.runOnce(()->driver.setRumble(RumbleType.kBothRumble, 1.0)))
            .andThen(shooterCommandFakeSubsystem.runOnce(()->{}))
            .withName("RevShoot");

        revPass = 
                Commands.runOnce(()->mostRecentAim = true)
            //.andThen(new IndependentCommand(track(new ShooterIndexerDefaultSpeed())))
            .andThen(
                new IndependentCommand(track(new ShooterTargetSpeed(
                    ()->Util.calculatePassVelocity(drivetrain) + leftFlywheelOffset,
                    ()->Util.calculatePassVelocity(drivetrain) + rightFlywheelOffset))))
            .andThen(new WaitUntilCommand(()->Shooter.getInstance().readyToShoot()))
            //.andThen(
            //     Commands.runOnce(()->driver.setRumble(RumbleType.kBothRumble, 1.0)))
            .andThen(shooterCommandFakeSubsystem.runOnce(()->{}))
            .withName("RevPass");


        testCommandChooser.setDefaultOption("None", Commands.none());
        testCommandChooser.addOption("Climb/ClimbUp", new ClimbUp());
        testCommandChooser.addOption("Climb/ClimbDown", new ClimbDown());
        testCommandChooser.addOption("Climb/SpoolUntilStall", new Spool());
        testCommandChooser.addOption("Climb/Unspool", new Unspool());
        testCommandChooser.addOption("Hood/AimToAngle[75°]", new AimToAngle(75.0));
        testCommandChooser.addOption("Hood/AimToAngle[60°]", new AimToAngle(60.0));
        testCommandChooser.addOption("Hood/AimToAngle[70°]", new AimToAngle(70.0));
        testCommandChooser.addOption("Hood/ZeroHood", new ZeroHood());
        testCommandChooser.addOption("Hood/HoodManualUp", new HoodManualUp());
        testCommandChooser.addOption("Hood/HoodManualDown", new HoodManualDown());
        testCommandChooser.addOption("Indexer/IndexerDefaultSpeed", new IndexerDefaultSpeed());
        testCommandChooser.addOption("Indexer/IndexerFullSpeed", new IndexerFullSpeed());
        testCommandChooser.addOption("Intake/DefaultIntake", new DefaultIntake());
        testCommandChooser.addOption("Intake/RunIntake", new RunIntake());
        testCommandChooser.addOption("Intake/EjectIntake", new EjectIntake());
        testCommandChooser.addOption("Intake/ExtendIntake", new ExtendIntake());
        testCommandChooser.addOption("Intake/RetractIntake", new RetractIntake());
        testCommandChooser.addOption("Intake/AgitateIntake", new AgitateIntake());
        testCommandChooser.addOption("Shooter/ShooterTargetSpeed[10]", new ShooterTargetSpeed(10.0));
        testCommandChooser.addOption("Shooter/ShooterTargetSpeed[" + Constants.HARDCODE_VELOCITY + "]", new ShooterTargetSpeed(Constants.HARDCODE_VELOCITY));
        testCommandChooser.addOption("Shooter/ShooterDefaultSpeed", new ShooterDefaultSpeed());
        testCommandChooser.addOption("ShooterIndexer/ShooterIndexerDefaultSpeed", new ShooterIndexerDefaultSpeed());
        testCommandChooser.addOption("ShooterIndexer/ShooterIndexerFullSpeed", new ShooterIndexerFullSpeed());

        SmartDashboard.putData("Test a Command", testCommandChooser);
        SmartDashboard.putData(CommandScheduler.getInstance());

        NamedCommands.registerCommand("ExtendIntake", track(Commands.runEnd(
            ()->IntakeExtension.getInstance().setVoltage(Volts.of(Constants.IntakeExtension.EXTENDING_VOLTAGE)),
            ()->IntakeExtension.getInstance().setVoltage(Volts.of(Constants.IntakeExtension.HOLDING_EXTEND_VOLTAGE)))));
        NamedCommands.registerCommand("RetractIntake", track(new RetractIntake().alongWith(new IndependentCommand(new RunIntake()))));
        NamedCommands.registerCommand("StartRunIntake", new IndependentCommand(track(new RunIntake())));
        NamedCommands.registerCommand("StartDefaultIntake", new IndependentCommand(track(new DefaultIntake())));
        NamedCommands.registerCommand("EjectIntake (with timeout)", track(new EjectIntake().withTimeout(1.0)));
        NamedCommands.registerCommand("HardShoot", track(hardShoot));
        NamedCommands.registerCommand("RevShoot", track(Commands.runOnce(()->{System.out.println(Util.calculateShootVelocity(drivetrain));})
            .andThen(Commands.runOnce(()->CommandScheduler.getInstance().schedule(new ShooterTargetSpeed(
                ()->Util.calculateShootVelocity(drivetrain) + leftFlywheelOffset,
                ()->Util.calculateShootVelocity(drivetrain) + rightFlywheelOffset))))));
        NamedCommands.registerCommand("InOutIntake", track(
            new IndependentCommand(track(new RunIntake()))
                        .alongWith(
                                IntakeExtension.getInstance()
                                        .runOnce(() -> IntakeExtension.getInstance()
                                                .setVoltage(Volts.of(Constants.IntakeExtension.RETRACTING_VOLTAGE)))
                                        .withTimeout(2.0)
                                        .andThen(new ExtendIntake().withTimeout(1.0))
                                        .andThen(IntakeExtension.getInstance()
                                                .runOnce(() -> IntakeExtension.getInstance().setVoltage(
                                                        Volts.of(Constants.IntakeExtension.RETRACTING_VOLTAGE)))
                                                .withTimeout(2.0))
                                        .andThen(new ExtendIntake().withTimeout(1.0))
                                        .andThen(IntakeExtension.getInstance()
                                                .runOnce(() -> IntakeExtension.getInstance().setVoltage(
                                                        Volts.of(Constants.IntakeExtension.RETRACTING_VOLTAGE)))
                                                .withTimeout(2.0)))));

        NamedCommands.registerCommand("Shoot", 
            track(new IndependentCommand(track(new AimToAngle(()->Util.calculateShootPitch(drivetrain).in(Degrees))))
            .alongWith(new IndependentCommand(track(new ShooterTargetSpeed(()->Util.calculateShootVelocity(drivetrain)))))
            .andThen(new WaitUntilCommand(()->Shooter.getInstance().readyToShoot() && Hood.getInstance().readyToShoot()))
            .andThen(new ShooterIndexerFullSpeed().alongWith(new IndexerFullSpeed())) // load to shoot
            .finallyDo(()->{
                CommandScheduler.getInstance().schedule(stow.get());
            }))
        .withName("Shoot"));
        
        autonChooser = AutoBuilder.buildAutoChooser();
        /*
        autonChooser = new SendableChooser<>();
        //autonChooser.setDefaultOption("None", Commands.none());
        //AutoBuilder.getAllAutoNames().forEach(s->autonChooser.addOption(s, new PathPlannerAuto(s)));
        PathPlannerAuto rightFerry = new PathPlannerAuto("Right Ferry");
        rightFerry.isRunning().onTrue(Commands.print("Right Ferry Auto Started")).onFalse(Commands.print("Right Ferry Auto Ended"));
        rightFerry.timeElapsed(1.0).onTrue(Commands.print("One Second Elapsed"));
        rightFerry.activePath("Right Ferry 1").onTrue(Commands.print("Starting Path \"Right Ferry 1\"")).onFalse(Commands.print("Ending Path \"Right Ferry 1\""));
        joystick.button(2).onTrue(rightFerry);
        autonChooser.addOption("Right Ferry", rightFerry);
        */
        SmartDashboard.putData("Auton Chooser", autonChooser);

        // ----------------------- DEFAULT BINDINGS HERE -------------------------

        Intake.getInstance().setDefaultCommand(new DefaultIntake());
        Indexer.getInstance().setDefaultCommand(new IndexerDefaultSpeed());
        ShooterIndexer.getInstance().setDefaultCommand(new ShooterIndexerDefaultSpeed());
        Shooter.getInstance().setDefaultCommand(new ShooterDefaultSpeed());
        // Hood.getInstance().setDefaultCommand(new AimToAngle(75.0));

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


    private void configureDebugBindings()
    {

        driver.x().whileTrue(new Spool());
        driver.y().whileTrue(new ClimbUp());
        // driver.a().whileTrue(new ClimbDown());
        driver.b().whileTrue(new Unspool());

        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> 
                        drive.withVelocityX(-driver.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                        .withVelocityY(-driver.getLeftX() * MaxSpeed * (isSlow ? Constants.TRANSLATION_SLOW_MULTIPLIER : 1.0)) // Drive left with negative X (left)
                        .withRotationalRate(-driver.getRightX() * MaxAngularRate * (isSlow ? Constants.ROTATION_SLOW_MULTIPLIER : 1.0)) // Drive counterclockwise with negative X (left)
                    ).withName("SwerveManual"));
    }

    private void configureDriverBindings() 
    {
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

        driver.rightTrigger().and(()->!mostRecentAim).whileTrue(track(
            shoot
            /*new RotateToAngle(drivetrain, ()->AlignConstants.HUB)
            .andThen(

            new AimToAngle(()->Telemetry.getInstance().getHoodAngle())
            .alongWith(new ShooterTargetSpeed(()->Telemetry.getInstance().getShooterSpeed()))
            .alongWith(new WaitCommand(2.0).andThen(new ShooterIndexerFullSpeed()
                .alongWith(new IndexerFullSpeed()))))
            .finallyDo(()->CommandScheduler.getInstance().schedule(stow.get()))*/
            ));

        driver.rightTrigger().and(()->mostRecentAim).whileTrue(track(pass));

        // tested in sim
        driver.leftBumper().onTrue(stow.get().andThen(Commands.print("Stowing")).withName("Stow"));

        // tested in sim
        //changed from retract/extand hopper and intake
        driver.rightBumper().onTrue(track(
            Commands.runOnce(()->{
                if (intakeTriggered)
                {
                    intakeTriggered = false;
                    CommandScheduler.getInstance().schedule(
                        track(new DefaultIntake()
                        .withName("DeactivateIntake")));
                }
                else if (intakeExtended)
                {
                    intakeTriggered = true;
                    CommandScheduler.getInstance().schedule(
                        track(new RunIntake()
                        .withName("ActivateIntake")));
                }
        })));
        
        
        // tested in sim
        driver.button(7) // home button/left paddle
            .onTrue(track(revPass));
        
        // tested in sim
        driver.button(8) // menu button/right paddle
            .onTrue(track(revShoot));

        // tested in sim
        // THIS ALL NEEDS TO BE CHANGED BASED ON WHAT DRIVE TEAM WANTS
        // driver.y().onTrue(track(new ClimbToLevel(3).andThen(new RunClimb())
        //     .withName("Climb L3"))); // TODO: add autoalign
        // // tested in sim
        // driver.b().onTrue(track(new ClimbToLevel(1).andThen(new RunClimb())
        //     .withName("Climb L1"))); // TODO: add autoalign

        // tested in sim
        driver.x().whileTrue(track(hardShoot));
        
        // tested (?) in sim
        driver.a().onTrue(track(new Unspool().andThen(stow.get()).withName("Drop")));
        
        // tested in sim
        driver.povUp().onTrue(Commands.runOnce(()->pitchOffset += Constants.PITCH_OFFSET_UNIT));
        driver.povDown().onTrue(Commands.runOnce(()->pitchOffset -= Constants.PITCH_OFFSET_UNIT));

        // tested in sim
        driver.povLeft().onTrue(Commands.runOnce(()->{
            leftFlywheelOffset -= Constants.FLYWHEEL_OFFSET_UNIT;
            rightFlywheelOffset -= Constants.FLYWHEEL_OFFSET_UNIT;}));
        driver.povRight().onTrue(Commands.runOnce(()->{
            leftFlywheelOffset += Constants.FLYWHEEL_OFFSET_UNIT;
            rightFlywheelOffset += Constants.FLYWHEEL_OFFSET_UNIT;}));

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
                drivetrain.applyRequest(() -> idle).ignoringDisable(true).withName("Drivetrain Set Idle"));
    }

    public void configureOperatorBindings()
    {
        operator.start().onTrue(track(
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
                .withName("ZeroDrivetrain")));

        operator.back().onTrue(track(new ZeroHood()
            //.alongWith(new ShooterDefaultSpeed())
            .withName("ZeroHood+Shooter")));

        operator.leftTrigger().whileTrue(track(new EjectIntake().finallyDo(()->CommandScheduler.getInstance().schedule(track(new RunIntake())))
            .withName("EjectIntake")));

        operator.rightTrigger().whileTrue(track(new IndependentCommand(track(new ShooterTargetSpeed(Constants.Shooter.SOFT_PASS_VELOCITY)))
            .andThen(new IndependentCommand(new AimToAngle(60.0)))
            .andThen(new IndependentCommand(track(new IndexerFullSpeed())))
            .andThen(new ShooterIndexerFullSpeed())
            .andThen(stow.get())
            .withName("SoftPass")));

        operator.leftBumper().onTrue(Commands.runOnce(()->
        {
            if (direction == PassDirection.Left) direction = PassDirection.Automatic;
            else direction = PassDirection.Left;
        }));
        operator.rightBumper().onTrue(Commands.runOnce(()->
        {
            if (direction == PassDirection.Right) direction = PassDirection.Automatic;
            else direction = PassDirection.Right;
        }));

        operator.y().whileTrue(track(new HoodManualUp()));
        operator.x().onTrue(track(new IndependentCommand(track(Intake.getInstance().run(()->Intake.getInstance().setVelocity(
                RotationsPerSecond.of(Constants.Intake.REDUCED_INTAKE_VELOCITY)))))
            .andThen(Commands.runOnce(()->intakeTriggered = true))
            .andThen(new RetractIntake())
            .andThen(Commands.runOnce(()->
            {
                intakeExtended = false;
            }
            )).withName("RetractIntake")));
        operator.a().whileTrue(track(new HoodManualDown()));
        operator.b().onTrue(track(new IndependentCommand(track(new DefaultIntake()))
            .andThen(Commands.runOnce(()->intakeTriggered = false))
            .andThen(new ExtendIntake())
            .andThen(Commands.runOnce(()->
            {
                intakeExtended = true;
            }
            )).withName("ExtendIntake")));

        operator.povUp().onTrue(Commands.runOnce(()->rightFlywheelOffset += Constants.FLYWHEEL_OFFSET_UNIT));
        operator.povDown().onTrue(Commands.runOnce(()->leftFlywheelOffset -= Constants.FLYWHEEL_OFFSET_UNIT));
        operator.povLeft().onTrue(Commands.runOnce(()->leftFlywheelOffset += Constants.FLYWHEEL_OFFSET_UNIT));
        operator.povRight().onTrue(Commands.runOnce(()->rightFlywheelOffset -= Constants.FLYWHEEL_OFFSET_UNIT));
    }

    public AlignDirection getAlignDirection ()
    {
        return alignDirection;
    }

    public void setAlignDirection (AlignDirection direction)
    {
        this.alignDirection = direction;
    } 

    public Command track(Command command)
    {
        commands.add(command);
        return command;
    }

    public Command getAutonomousCommand() 
    {
        return autonChooser.getSelected();
    }
}
