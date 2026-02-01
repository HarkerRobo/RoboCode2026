// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.Simulation;
import frc.robot.Constants.TunerConstants;
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
   private static final Drivetrain drivetrain = Modules.createDrivetrain();

   private CommandXboxController driver = new CommandXboxController(0);

   private RobotContainer() 
   {
      intake.setDefaultCommand(new DefaultIntake());

      configureBindings();
   }

   private final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
   private final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

   private final SwerveRequest.FieldCentric m_driveRequest = new SwerveRequest.FieldCentric()
         .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
         .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
         .withSteerRequestType(SteerRequestType.Position);

   private void configureBindings() 
   {
      // driver.button(1).onTrue(new ZeroHood());
      // driver.button(2).onTrue(new ZeroHoodSoft());
      // driver.button(3).onTrue(new AimToAngle(60.0));

      drivetrain.setDefaultCommand(
         drivetrain.applyRequest(() -> 
             m_driveRequest.withVelocityX(-driver.getLeftY() * MaxSpeed)
               .withVelocityY(-driver.getLeftX() * MaxSpeed)
               .withRotationalRate(-driver.getRightX() * MaxAngularRate)
         )
      );

      /*
      driver.button(1).onTrue(new ExtendHopper());
      driver.button(2).onTrue(new RetractHopper());
      */
      /*
      driver.button(1).onTrue(Indexer.getInstance().sysIdQuasistatic(SysIdRoutine.Direction.kForward));
      driver.button(2).onTrue(Indexer.getInstance().sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
      driver.button(3).onTrue(Indexer.getInstance().sysIdDynamic(SysIdRoutine.Direction.kForward));
      driver.button(4).onTrue(Indexer.getInstance().sysIdDynamic(SysIdRoutine.Direction.kReverse));
      */
      /*
      driver.button(1).onTrue(Hopper.getInstance().sysIdQuasistatic(SysIdRoutine.Direction.kForward));
      driver.button(2).onTrue(Hopper.getInstance().sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
      driver.button(3).onTrue(Hopper.getInstance().sysIdDynamic(SysIdRoutine.Direction.kForward));
      driver.button(4).onTrue(Hopper.getInstance().sysIdDynamic(SysIdRoutine.Direction.kReverse));
      */
      /*
      driver.button(1).onTrue(new ExtendHopper());
      driver.button(2).onTrue(new RetractHopper());
      */
      /*
      driver.button(1).onTrue(Shooter.getInstance().sysIdQuasistatic(SysIdRoutine.Direction.kForward));
      driver.button(2).onTrue(Shooter.getInstance().sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
      driver.button(3).onTrue(Shooter.getInstance().sysIdDynamic(SysIdRoutine.Direction.kForward));
      driver.button(4).onTrue(Shooter.getInstance().sysIdDynamic(SysIdRoutine.Direction.kReverse));
      */
      /*
      driver.button(1).whileTrue(new RampUpShooter());
      driver.button(2).whileTrue(new ResetShooter());
      */
      /*
      driver.button(1).onTrue(Intake.getInstance().sysIdQuasistatic(SysIdRoutine.Direction.kForward));
      driver.button(2).onTrue(Intake.getInstance().sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
      driver.button(3).onTrue(Intake.getInstance().sysIdDynamic(SysIdRoutine.Direction.kForward));
      driver.button(4).onTrue(Intake.getInstance().sysIdDynamic(SysIdRoutine.Direction.kReverse));
      */

 
      /*
      driver.button(1).onTrue(new AimToAngle(30.0));
      driver.button(2).onTrue(new AimToAngle(60.0));
      driver.button(3).onTrue(new AimToAngle(0.0));
      */
      /*
      driver.button(1).onTrue(Hood.getInstance().sysIdQpuasistatic(SysIdRoutine.Direction.kForward));
      driver.button(2).onTrue(Hood.getInstance().sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
      driver.button(3).onTrue(Hood.getInstance().sysIdDynamic(SysIdRoutine.Direction.kForward));
      driver.button(4).onTrue(Hood.getInstance().sysIdDynamic(SysIdRoutine.Direction.kReverse));
      */
      
      /*
      driver.button(1).onTrue(Commands.runOnce(()->SimulationState.getInstance().testPlop(FieldLocation.BlueHub)));
      driver.button(2).onTrue(Commands.runOnce(()->SimulationState.getInstance().testPlop(FieldLocation.RedHub)));
      driver.button(3).onTrue(Commands.runOnce(()->SimulationState.getInstance().spawnFromOutpost(Alliance.Blue)));
      driver.button(4).onTrue(Commands.runOnce(()->SimulationState.getInstance().spawnFromOutpost(Alliance.Red)));
      driver.button(5).onTrue(Commands.runOnce(()->SimulationState.getInstance().testDrop()));
      */
      /*
      driver.button(1).onTrue(new AimToAngle(60.0, 10.0));
      driver.button(2).onTrue(new AimToAngle(120.0, 5.0));
      driver.button(3).onTrue(new AimToAngle(180.0, 20.0));
      driver.button(4).onTrue(new AimToAngle(330.0, 15.0));
      driver.button(5).onTrue(new AimToAngle(750.0, 15.0));
      */
   }

   public Command getAutonomousCommand() 
   {
      return Commands.print("No autonomous command configured");
   }

   public static Drivetrain getDrivetrain() {
      return drivetrain;
   }

   public static RobotContainer getInstance ()
   {
      if (instance == null) instance = new RobotContainer();

      return instance;
   }
}
