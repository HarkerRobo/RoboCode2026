// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.*;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.TunerConstants;
import frc.robot.commands.climb.ClimbToLevel;
import frc.robot.commands.climb.MoveDownUntilStall;
import frc.robot.commands.climb.RunClimb;
import frc.robot.commands.climb.UndeployClimb;
import frc.robot.commands.drive.DriveToPose;
import frc.robot.commands.hood.AimToAngle;
import frc.robot.commands.hood.ZeroHood;
import frc.robot.commands.hood.ZeroHoodSoft;
import frc.robot.commands.hopper.ExtendHopper;
import frc.robot.commands.hopper.RetractHopper;
import frc.robot.commands.indexer.IndexerDefaultSpeed;
import frc.robot.commands.indexer.IndexerFullSpeed;
import frc.robot.commands.intake.DefaultIntake;
import frc.robot.commands.intake.EjectIntake;
import frc.robot.commands.intake.ExtendIntake;
import frc.robot.commands.intake.RetractIntake;
import frc.robot.commands.intake.RunIntake;
import frc.robot.commands.shooter.ShooterDefaultSpeed;
import frc.robot.commands.shooter.ShooterTargetSpeed;
import frc.robot.subsystems.*;
import frc.robot.subsystems.intake.Intake;
import frc.robot.util.Util;



public class RobotContainer 
{
    private final Intake intake = Intake.getInstance();

    public double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    public final CommandXboxController driver = new CommandXboxController(0);
    public final CommandXboxController operator = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = Modules.createDrivetrain();

    public SendableChooser<Command> testCommandChooser = new SendableChooser<>();

    private SendableChooser<Command> autonChooser;

    private boolean isSlow = false;
    public boolean mostRecentAim = false; // false = shoot; true = pass
    private boolean intakeTriggered = false; // true if intake has been enabled
    private boolean intakeExtended = false; //true if intake and hopper have been extended
    public double pitchOffset = 0.0;
    private double flywheelOffset = 0.0;
  
    private List<Command> commands = new ArrayList<>(40);
  
    public static enum PassDirection {Left, Right, Automatic};

    private PassDirection direction = PassDirection.Left; // Default

    public void setPassDirection(PassDirection newDirection) {
        direction = newDirection;
    }

    public PassDirection getPassDirection() {
        return direction;
    }

    public Command SetPassDirection(PassDirection direct) {
        return new Command() {
            public void execute() {setPassDirection(direct);}
        };
    }
  
    public boolean onLeftSize()
    {
      if (direction == PassDirection.Left) return true;
      if (direction == PassDirection.Right) return false;
      return Util.onLeftSide(drivetrain);
    }
  
        
    public RobotContainer() 
    {
        testCommandChooser.setDefaultOption("None", Commands.none());
        testCommandChooser.addOption("Climb/ClimbToLevel[1]", new ClimbToLevel(1));
        testCommandChooser.addOption("Climb/ClimbToLevel[2]", new ClimbToLevel(2));
        testCommandChooser.addOption("Climb/ClimbToLevel[3]", new ClimbToLevel(3));
        testCommandChooser.addOption("Climb/MoveDownUntilStall", new MoveDownUntilStall());
        testCommandChooser.addOption("Hood/AimToAngle[" + Constants.Hood.MIN_ANGLE + "°]", new AimToAngle(Constants.Hood.MIN_ANGLE));
        testCommandChooser.addOption("Hood/AimToAngle[10°]", new AimToAngle(10.0));
        testCommandChooser.addOption("Hood/AimToAngle[20°]", new AimToAngle(20.0));
        testCommandChooser.addOption("Hood/AimToAngle[" + Constants.Hood.MAX_ANGLE + "°]", new AimToAngle(Constants.Hood.MAX_ANGLE));
        testCommandChooser.addOption("Hood/ZeroHood", new ZeroHood());
        testCommandChooser.addOption("Hood/ZeroHoodSoft", new ZeroHoodSoft());
        testCommandChooser.addOption("Hood/ExtendHopper", new ExtendHopper());
        testCommandChooser.addOption("Hood/RetractHopper", new RetractHopper());
        testCommandChooser.addOption("Indexer/IndexerDefaultSpeed", new IndexerDefaultSpeed());
        testCommandChooser.addOption("Indexer/IndexerFullSpeed", new IndexerFullSpeed());
        testCommandChooser.addOption("Intake/DefaultIntake", new DefaultIntake());
        testCommandChooser.addOption("Intake/RunIntake", new RunIntake());
        testCommandChooser.addOption("Intake/EjectIntake", new EjectIntake());
        testCommandChooser.addOption("Intake/ExtendIntake", new ExtendIntake());
        testCommandChooser.addOption("Intake/RetractIntake", new RetractIntake());
        testCommandChooser.addOption("Shooter/ShooterTargetSpeed[10]", new ShooterTargetSpeed(10.0));
        testCommandChooser.addOption("Shooter/ShooterTargetSpeed[" + Constants.HARDCODE_VELOCITY + "]", new ShooterTargetSpeed(Constants.HARDCODE_VELOCITY));
        testCommandChooser.addOption("Shooter/ShooterDefaultSpeed", new ShooterDefaultSpeed());

        SmartDashboard.putData("Test a Command", testCommandChooser);
        SmartDashboard.putData(CommandScheduler.getInstance());

        NamedCommands.registerCommand("ExtendIntake", new ExtendIntake());
        NamedCommands.registerCommand("RetractIntake", new RetractIntake());
        NamedCommands.registerCommand("StartRunIntake", Intake.getInstance().runOnce(()->Intake.getInstance().setMainVoltage(Volts.of(Constants.Intake.INTAKE_VOLTAGE))));
        NamedCommands.registerCommand("StartDefaultIntake", Intake.getInstance().runOnce(()->Intake.getInstance().setMainVoltage(Volts.of(Constants.Intake.DEFAULT_INTAKE_VOLTAGE))));
        NamedCommands.registerCommand("DefaultIntake", new DefaultIntake());
        NamedCommands.registerCommand("EjectIntake (with timeout)", new EjectIntake().withTimeout(1.0));

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

        intake.setDefaultCommand(new DefaultIntake());
        Indexer.getInstance().setDefaultCommand(new IndexerDefaultSpeed());

        boolean useDebuggingBindings = false; // mainly for sysid or debugging
        boolean useDefaultBindings = false; // in case ever the official controls don't work, use these as a backup to be able to drive around
        if (useDebuggingBindings) configureDebugBindings();
        else if (useDefaultBindings)
        {
            configureDefaultBindings();
        }
        else
        {
            configureDriverBindings();
            configureOperatorBindings();
        }
        
        drivetrain.registerTelemetry(Telemetry.getInstance()::telemeterize);
    }

    private void configureDebugBindings()
    {
        driver.button(1).onTrue(Indexer.getInstance().sysIdQuasistatic(Direction.kForward));
        driver.button(2).onTrue(Indexer.getInstance().sysIdQuasistatic(Direction.kReverse));
        driver.button(3).onTrue(Indexer.getInstance().sysIdDynamic(Direction.kForward));
        driver.button(4).onTrue(Indexer.getInstance().sysIdDynamic(Direction.kReverse));

        driver.povUp().onTrue(Commands.print("POV UP"));
        driver.povDown().onTrue(Commands.print("POV DOWN"));
        driver.povLeft().onTrue(Commands.print("POV LEFT"));
        driver.povRight().onTrue(Commands.print("POV RIGHT"));
        
    }

    private void configureDefaultBindings()
    {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> 
                        drive.withVelocityX(-driver.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                        .withVelocityY(-driver.getLeftX() * MaxSpeed * (isSlow ? Constants.TRANSLATION_SLOW_MULTIPLIER : 1.0)) // Drive left with negative X (left)
                        .withRotationalRate(-driver.getRightX() * MaxAngularRate * (isSlow ? Constants.ROTATION_SLOW_MULTIPLIER : 1.0)) // Drive counterclockwise with negative X (left)
                    ).withName("SwerveManual"));

        driver.b().toggleOnTrue(new RunIntake());
        driver.a().toggleOnTrue(new ExtendIntake());
        driver.y().toggleOnTrue(new RetractIntake());

        driver.start().onTrue(
                drivetrain.runOnce(() -> drivetrain.seedFieldCentric())
                .andThen(drivetrain.runOnce(() -> drivetrain.resetPose(
                            (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) ? 
                            FlippingUtil.flipFieldPose(Constants.ZEROING_POSE) : Constants.ZEROING_POSE)))
                .withName("ZeroDrivetrain"));
    }
   

    private void configureDriverBindings() 
    {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> 
                        drive.withVelocityX(-driver.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                        .withVelocityY(-driver.getLeftX() * MaxSpeed * (isSlow ? Constants.TRANSLATION_SLOW_MULTIPLIER : 1.0)) // Drive left with negative X (left)
                        .withRotationalRate(-driver.getRightX() * MaxAngularRate * (isSlow ? Constants.ROTATION_SLOW_MULTIPLIER : 1.0)) // Drive counterclockwise with negative X (left)
                    ).withName("SwerveManual"));

        // tested
        driver.leftTrigger().whileTrue(new StartEndCommand(()->isSlow = true, ()->isSlow = false).withName("ToggleSlow"));

        // tested in sim
        Supplier<Command> stow = ()->Commands.runOnce(()->CommandScheduler.getInstance().cancel(commands.toArray(new Command[0]))).andThen(
            new ShooterDefaultSpeed()); // because a command instance cannot be scheduled to independent triggers

        // tested in sim
        Command shoot = 
        new DriveToPose(drivetrain, ()->AlignConstants.HUB)
            .alongWith(new AimToAngle(()->Util.calculateShootPitch(drivetrain).in(Degrees) + pitchOffset))
        .andThen(new WaitUntilCommand(()->Shooter.getInstance().readyToShoot(Util.calculateShootVelocity(drivetrain)) && Hood.getInstance().readyToShoot()))
        .andThen(new IndexerFullSpeed()) // load to shoot
        .finallyDo(()->{
            CommandScheduler.getInstance().schedule(stow.get());
        })
        .withName("Shoot");
        
        // tested in sim
        Command pass = 
        new DriveToPose(drivetrain, ()->onLeftSize() ? 
            Constants.PASS_LEFT_TARGET_POSITION.toTranslation2d() : 
            Constants.PASS_RIGHT_TARGET_POSITION.toTranslation2d())
            .alongWith(new AimToAngle(()->Util.calculatePassPitch(drivetrain).in(Degrees) + pitchOffset))
        .andThen(new WaitUntilCommand(()->Shooter.getInstance().readyToShoot(Util.calculatePassVelocity(drivetrain)) && Hood.getInstance().readyToShoot()))
        .andThen(new IndexerFullSpeed()) // load to shoot
        .finallyDo(()->{
            CommandScheduler.getInstance().schedule(stow.get());
        })
        .withName("Pass");

        driver.rightTrigger().and(()->!mostRecentAim).whileTrue(track(shoot));
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
        // driver.rightBumper().onTrue(track(
        //     Commands.runOnce(()->{
        //         if (intakeTriggered)
        //         {
        //             intakeTriggered = false;
        //             CommandScheduler.getInstance().schedule(
        //                 track(new DefaultIntake().withTimeout(0.01).andThen(new RetractIntake().alongWith(new RetractHopper()))
        //                 .withName("UndeployIntake")));
        //         }
        //         else
        //         {
        //             intakeTriggered = true;
        //             CommandScheduler.getInstance().schedule(
        //                 track(new ExtendIntake().alongWith(new ExtendHopper()).andThen(new RunIntake())
        //                 .withName("DeployIntake")));
        //         }
        // })));
        
        
        // tested in sim
        driver.button(7) // home button/left paddle
            .onTrue(track(new ShooterTargetSpeed(()->Util.calculateShootVelocity(drivetrain) + flywheelOffset).until(()->Shooter.getInstance().readyToShoot())
            .andThen(
                Commands.runOnce(()->driver.setRumble(RumbleType.kBothRumble, 1.0)))
                .andThen(Commands.runOnce(()->mostRecentAim = false))
                .withName("RevShoot")));
        
        // tested in sim
        driver.button(8) // menu button/right paddle
            .onTrue(track(new ShooterTargetSpeed(()->Util.calculatePassVelocity(drivetrain) + flywheelOffset).until(()->Shooter.getInstance().readyToShoot())
            .andThen(
                Commands.runOnce(()->driver.setRumble(RumbleType.kBothRumble, 1.0)))
                .andThen(Commands.runOnce(()->mostRecentAim = true))
                .withName("RevPass")));

        // tested in sim
        driver.y().onTrue(track(new ClimbToLevel(3).andThen(new RunClimb())
            .withName("Climb L3"))); // TODO: add autoalign
        // tested in sim
        driver.b().onTrue(track(new ClimbToLevel(1).andThen(new RunClimb())
            .withName("Climb L1"))); // TODO: add autoalign

        // tested in sim
        driver.x().onTrue(track(
            new AimToAngle(Constants.HARDCODE_HOOD_PITCH.in(Degrees) + pitchOffset)
            .alongWith(new ShooterTargetSpeed(()->Constants.HARDCODE_VELOCITY).until(()->Shooter.getInstance().readyToShoot()))
            .andThen(new IndexerFullSpeed())
            .withName("HardShoot")));
        
        // tested (?) in sim
        driver.a().onTrue(track(new UndeployClimb().andThen(stow.get()).withName("Drop")));
        
        // tested in sim
        driver.povUp().onTrue(Commands.runOnce(()->pitchOffset += Constants.PITCH_OFFSET_UNIT));
        driver.povDown().onTrue(Commands.runOnce(()->pitchOffset -= Constants.PITCH_OFFSET_UNIT));

        // tested in sim
        driver.povLeft().onTrue(Commands.runOnce(()->flywheelOffset += Constants.FLYWHEEL_OFFSET_UNIT));
        driver.povRight().onTrue(Commands.runOnce(()->flywheelOffset -= Constants.FLYWHEEL_OFFSET_UNIT));

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
                drivetrain.applyRequest(() -> idle).ignoringDisable(true).withName("Drivetrain Set Idle"));

                /*
        // Zero DT
        operator.x().onTrue(
        drivetrain.runOnce(() -> drivetrain.seedFieldCentric())
                .andThen(drivetrain.runOnce(() -> drivetrain.resetPose(
                            (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) ? 
                            FlippingUtil.flipFieldPose(Constants.ZEROING_POSE) : Constants.ZEROING_POSE)))
                .withName("ZeroDrivetrain"));
                */
    }

    public void configureOperatorBindings()
    {
        operator.start().onTrue(
                drivetrain.runOnce(() -> drivetrain.seedFieldCentric())
                .andThen(drivetrain.runOnce(() -> drivetrain.resetPose(
                            (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) ? 
                            FlippingUtil.flipFieldPose(Constants.ZEROING_POSE) : Constants.ZEROING_POSE)))
                .withName("ZeroDrivetrain"));

        operator.back().onTrue(new ZeroHood()
            .alongWith(new ShooterDefaultSpeed()));

        operator.leftTrigger().onTrue(new EjectIntake());
        // operator.rightTrigger().onTrue();    Soft Pass

        operator.leftBumper().onTrue(SetPassDirection(PassDirection.Left));
        operator.rightBumper().onTrue(SetPassDirection(PassDirection.Right));

        operator.y().onTrue(new AimToAngle(Constants.Hood.MAX_ANGLE));
        operator.x().onTrue(new RetractIntake()
            .andThen(Commands.runOnce(()->
            {
                intakeExtended = false;
            }
            )));
        operator.a().onTrue(new AimToAngle(Constants.Hood.MIN_ANGLE));
        operator.b().onTrue(new ExtendIntake()
            .andThen(Commands.runOnce(()->
            {
                intakeExtended = true;
            })));

        operator.povUp().onTrue(Shooter.getInstance().runOnce(() -> 
            Shooter.getInstance().setRightVelocity(RotationsPerSecond.of(
                Math.min(Constants.Shooter.MAX_VELOCITY, 
                Constants.Shooter.INCREASE_VELOCITY + Shooter.getInstance().getRightVelocity().in(RotationsPerSecond))))));
        
        operator.povDown().onTrue(Shooter.getInstance().runOnce(() -> 
            Shooter.getInstance().setLeftVelocity(RotationsPerSecond.of(
                Math.max(Constants.Shooter.DEFAULT_VELOCITY, 
                -Constants.Shooter.INCREASE_VELOCITY + Shooter.getInstance().getLeftVelocity().in(RotationsPerSecond))))));      
        
        operator.povLeft().onTrue(Shooter.getInstance().runOnce(() -> 
            Shooter.getInstance().setLeftVelocity(RotationsPerSecond.of(
                Math.min(Constants.Shooter.MAX_VELOCITY, 
                Constants.Shooter.INCREASE_VELOCITY + Shooter.getInstance().getLeftVelocity().in(RotationsPerSecond))))));
              
        operator.povRight().onTrue(Shooter.getInstance().runOnce(() -> 
            Shooter.getInstance().setRightVelocity(RotationsPerSecond.of(
                Math.max(Constants.Shooter.DEFAULT_VELOCITY, 
                -Constants.Shooter.INCREASE_VELOCITY + Shooter.getInstance().getRightVelocity().in(RotationsPerSecond))))));             
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
