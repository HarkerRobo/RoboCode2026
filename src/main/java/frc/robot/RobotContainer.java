// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.*;
import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.*;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.Drivetrain;
import frc.robot.Constants.Simulation;
<<<<<<< Updated upstream
import frc.robot.generated.TunerConstants;
=======
import frc.robot.Constants.TunerConstants;
import frc.robot.commands.climb.ClimbToLevel;
import frc.robot.commands.climb.ElevatorGoUp;
import frc.robot.commands.climb.ElevatorStop;
import frc.robot.commands.climb.MoveDownUntilStall;
>>>>>>> Stashed changes
import frc.robot.commands.hood.AimToAngle;
import frc.robot.commands.hood.ZeroHood;
import frc.robot.commands.hood.ZeroHoodSoft;
import frc.robot.commands.hopper.ExtendHopper;
import frc.robot.commands.hopper.RetractHopper;
import frc.robot.commands.intake.DefaultIntake;
import frc.robot.commands.intake.RunIntake;
import frc.robot.commands.shooter.RampUpShooter;
import frc.robot.commands.shooter.ResetShooter;
import frc.robot.simulation.SimulationState;
import frc.robot.simulation.SimulationState.FieldLocation;
import frc.robot.subsystems.*;



public class RobotContainer 
{
   private static RobotContainer instance;
   private static final Intake intake = Intake.getInstance();
   
   public double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
   private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

   /* Setting up bindings for necessary control of the swerve drive platform */
   private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
   private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
   private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

   private final CommandXboxController joystick = new CommandXboxController(0);

   public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

   private RobotContainer() 
   {
      intake.setDefaultCommand(new DefaultIntake());

      configureBindings();
   }


   private void configureBindings() 
   {
<<<<<<< Updated upstream
=======
      
      joystick.button(1).onTrue(new ClimbToLevel(1));
      joystick.button(2).onTrue(new ClimbToLevel(2));
      joystick.button(3).onTrue(new ClimbToLevel(3));
      joystick.button(4).onTrue(new MoveDownUntilStall());
      joystick.button(5).whileTrue(new ElevatorGoUp());
      joystick.button(6).onTrue(new ElevatorStop());
      // joystick.button(1).whileTrue(Climb.getInstance().sysIdQuasistatic(Direction.kForward));
      // joystick.button(2).whileTrue(Climb.getInstance().sysIdQuasistatic(Direction.kReverse));
      // joystick.button(3).whileTrue(Climb.getInstance().sysIdDynamic(Direction.kForward));
      // joystick.button(4).whileTrue(Climb.getInstance().sysIdDynamic(Direction.kReverse));
      

>>>>>>> Stashed changes
      // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
              
        drivetrain.registerTelemetry(Telemetry.getInstance()::telemeterize);

   }
       public Command getAutonomousCommand() {
        // Simple drive forward auton
        final var idle = new SwerveRequest.Idle();
        return Commands.sequence(
            // Reset our field centric heading to match the robot
            // facing away from our alliance station wall (0 deg).
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
            // Then slowly drive forward (away from us) for 5 seconds.
            drivetrain.applyRequest(() ->
                drive.withVelocityX(0.5)
                    .withVelocityY(0)
                    .withRotationalRate(0)
            )
            .withTimeout(5.0),
            // Finally idle for the rest of auton
            drivetrain.applyRequest(() -> idle)
        );
    }

   public static RobotContainer getInstance ()
   {
      if (instance == null) instance = new RobotContainer();

      return instance;
   }
}
