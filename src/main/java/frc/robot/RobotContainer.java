// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.Simulation;
import frc.robot.commands.AimToAngle;
import frc.robot.simulation.SimulationState;
import frc.robot.simulation.SimulationState.FieldLocation;
import frc.robot.subsystems.Turret;

public class RobotContainer 
{
   private static RobotContainer instance;


   private CommandXboxController driver = new CommandXboxController(0);

   private RobotContainer() 
   {
      configureBindings();
   }

   private void configureBindings() 
   {
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

   public static RobotContainer getInstance ()
   {
      if (instance == null) instance = new RobotContainer();

      return instance;
   }
}
