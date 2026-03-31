// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.hood.ZeroHood;
import frc.robot.simulation.SimulationState;
import frc.robot.util.Util;

/**
 * Main robot entry point that manages the robot mode lifecycle and scheduling
 */
public class Robot extends TimedRobot 
{
   private Command autonomousCommand;
   public RobotContainer robotContainer;
   public static Robot instance;



   public Robot() 
   {
      instance = this;
      Telemetry.getInstance();
      robotContainer = new RobotContainer();
      

      boolean logSimulation = false;

      // automatically saves log data for telemetry, driver station controls, and joystick presses
      if (isReal() || logSimulation)
      {
         DataLogManager.start();
         DriverStation.startDataLog(DataLogManager.getLog());
      }
   }
   /**
    * Called when the robot starts
    * Initiailizes RobotContainer bindings and constructs a LimelightSimulation if in simulation
    */
   @Override
   public void robotInit() 
   {
      robotContainer.init();
      Util.init();


      LimelightHelpers.setCameraPose_RobotSpace(Constants.Vision.kCamera1Name, 
      Constants.Vision.kRobotToCam1.getX(), Constants.Vision.kRobotToCam1.getY(), Constants.Vision.kRobotToCam1.getZ(),
      Units.radiansToDegrees(Constants.Vision.kRobotToCam1.getRotation().getX()), Units.radiansToDegrees(Constants.Vision.kRobotToCam1.getRotation().getY()), Units.radiansToDegrees(Constants.Vision.kRobotToCam1.getRotation().getZ()));
   

      LimelightHelpers.setCameraPose_RobotSpace(Constants.Vision.kCamera2Name, 
      Constants.Vision.kRobotToCam2.getX(), Constants.Vision.kRobotToCam2.getY(), Constants.Vision.kRobotToCam2.getZ(),
      Units.radiansToDegrees(Constants.Vision.kRobotToCam2.getRotation().getX()), Units.radiansToDegrees(Constants.Vision.kRobotToCam2.getRotation().getY()), Units.radiansToDegrees(Constants.Vision.kRobotToCam2.getRotation().getZ()));
   }
      

   @Override
   public void robotPeriodic() 
   {
      CommandScheduler.getInstance().run();

      CommandScheduler.getInstance().schedule(robotContainer.testCommandChooser.getSelected());

      Telemetry.getInstance().update();
   }
   /**
    * Called when entering disabled mode
    * Stops SignalLogger if on real hardware
    */
   @Override
   public void disabledInit() 
   {
      robotContainer.driver.setRumble(RumbleType.kBothRumble, 0.0);
      if(isReal())
      {
         SignalLogger.stop();
      }
   }
   /**
    * Called periodically while disabled
    * No actions are required
    */
   @Override
   public void disabledPeriodic() {}

   /**
    * Called when exiting disabled mode
    * Re-starts SignalLogger if on real hardware
    */
   @Override
   public void disabledExit() 
   {
      if (isReal())
      {
         SignalLogger.start();
      }
   }
   /**
    * Called when entering autonomous mode
    * Grabs the selected autonomous command and schedules it if it exists
    */
   @Override
   public void autonomousInit() 
   {
      autonomousCommand = robotContainer.getAutonomousCommand();

      if (autonomousCommand != null) 
      {
         CommandScheduler.getInstance().schedule(autonomousCommand);
      }
   }
   /**
    * Called periodically during autonomous mode
    * No logic is required
    */
   @Override
   public void autonomousPeriodic() {}

   /**
    * Called when exiting autonomous mode
    * No actions are required
    */
   @Override
   public void autonomousExit() {}
   /**
    * Called when entering teleoperated mode
    * If an autonomous command is still running, it is canceled
    */
   @Override
   public void teleopInit() 
   {
      CommandScheduler.getInstance().schedule(new ZeroHood());
      if (autonomousCommand != null) 
      {
         autonomousCommand.cancel();
      }
   }
   /**
    * Called periodically during teleoperated mode
    * No logic is required
    */
   @Override
   public void teleopPeriodic() {}
   /**
    * Called when exiting teleoperated mode
    * No actions are required
    */
   @Override
   public void teleopExit() {}
   /**
    * Called when entering test mode
    * All running commands are canceled
    */
   @Override
   public void testInit() 
   {
      CommandScheduler.getInstance().cancelAll();
   }
   /**
    * Called periodically during test mode
    * No logic is required
    */
   @Override
   public void testPeriodic() {}
   /**
    * Called when exiting test mode
    * All running commands are canceled
    */
   @Override
   public void testExit() 
   {
      CommandScheduler.getInstance().cancelAll();
   }
   /**
    * Called when entering simulation mode
    * No simulation initialization is required here
    */
   @Override
   public void simulationInit() 
   {
   }
   /**
    * Called periodically during simulation mode
    * Updates the simulation state each loop
    */
   @Override
   public void simulationPeriodic()
   {
      SimulationState.getInstance().update();
   }
}
