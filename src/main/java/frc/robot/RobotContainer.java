// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.*;
import static edu.wpi.first.units.Units.*;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.swerve.SwerveModule.*;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.Drivetrain;
import frc.robot.Constants.Simulation;
import frc.robot.Constants.TunerConstants;
import frc.robot.commands.climb.ClimbToLevel;
import frc.robot.commands.climb.ElevatorUp;
import frc.robot.commands.climb.MoveDownUntilStall;
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
import frc.robot.commands.shooter.RampUpShooter;
import frc.robot.commands.shooter.ResetShooter;
import frc.robot.simulation.SimulationState;
import frc.robot.simulation.SimulationState.FieldLocation;
import frc.robot.subsystems.*;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeExtension;



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

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = Modules.createDrivetrain();

    public SendableChooser<Command> testCommandChooser = new SendableChooser<>();

    private SendableChooser<Command> autonChooser;
        
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
        testCommandChooser.addOption("Shooter/RampUpShooter", new RampUpShooter());
        testCommandChooser.addOption("Shooter/ResetShooter", new ResetShooter());

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

        configureBindings();
    }

    private void configureBindings() 
    {
        intake.setDefaultCommand(new DefaultIntake());
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
                    ).withName("SwerveManual"));

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
                drivetrain.applyRequest(() -> idle).ignoringDisable(true).withName("Drivetrain Set Idle"));

                /*
        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake).withName("Drivetrain Brake"));
        joystick.b().whileTrue(drivetrain
                .applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));
                */
        
        joystick.x().toggleOnTrue(new RunIntake());
        joystick.y().toggleOnTrue(new ExtendIntake());
        joystick.a().toggleOnTrue(new RetractIntake());

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        /*
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        */
        joystick.start().whileTrue(new DriveToPose(drivetrain));

        // Zero DT
        joystick.leftBumper().onTrue(
                drivetrain.runOnce(() -> drivetrain.seedFieldCentric())
                .andThen(drivetrain.runOnce(() -> drivetrain.resetPose(
                            (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) ? 
                            FlippingUtil.flipFieldPose(Constants.ZEROING_POSE) : Constants.ZEROING_POSE)))
                .withName("ZeroDrivetrain"));
        drivetrain.registerTelemetry(Telemetry.getInstance()::telemeterize);
    }

    public Command getAutonomousCommand() 
    {
        return autonChooser.getSelected();
    }
}
