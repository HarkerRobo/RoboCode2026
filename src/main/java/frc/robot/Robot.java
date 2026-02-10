// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.Simulation;
import frc.robot.simulation.SimulationState;

public class Robot extends TimedRobot 
{
   Command autonomousCommand;

   public Robot() 
   {
      Telemetry.getInstance();
      RobotContainer.getInstance();
      

      boolean logSimulation = false;

      // automatically saves log data for telemetry, driver station controls, and joystick presses
      if (isReal() || logSimulation)
      {
         DataLogManager.start();
         DriverStation.startDataLog(DataLogManager.getLog());
      }
   }

   @Override
   public void robotPeriodic() 
   {
      CommandScheduler.getInstance().run();

      CommandScheduler.getInstance().schedule(RobotContainer.getInstance().testCommandChooser.getSelected());

      Telemetry.getInstance().update();
   }

   @Override
   public void disabledInit() 
   {
      if(isReal())
      {
         SignalLogger.stop();
      }
   }

   @Override
   public void disabledPeriodic() {}

   @Override
   public void disabledExit() 
   {
      if (isReal())
      {
         SignalLogger.start();
      }
   }

   @Override
   public void autonomousInit() 
   {
      autonomousCommand = RobotContainer.getInstance().getAutonomousCommand();

      if (autonomousCommand != null) 
      {
         CommandScheduler.getInstance().schedule(autonomousCommand);
      }
   }

   @Override
   public void autonomousPeriodic() {}

   @Override
   public void autonomousExit() {}

   @Override
   public void teleopInit() 
   {
      if (autonomousCommand != null) 
      {
         autonomousCommand.cancel();
      }
   }

   @Override
   public void teleopPeriodic() {}

   @Override
   public void teleopExit() {}

   @Override
   public void testInit() 
   {
      CommandScheduler.getInstance().cancelAll();
   }

   @Override
   public void testPeriodic() {}

   @Override
   public void testExit() 
   {
      CommandScheduler.getInstance().cancelAll();
   }

   @Override
   public void simulationInit() 
   {
   }

   @Override
   public void simulationPeriodic()
   {
      SimulationState.getInstance().update();
   }
}
