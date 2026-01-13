// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AimToAngle;
import frc.robot.subsystems.Turret;

public class RobotContainer 
{
   private static RobotContainer instance;

   private static CommandXboxController driver = new CommandXboxController(0);

   private RobotContainer() 
   {
      configureBindings();
   }

   private void configureBindings() 
   {
      /*
      driver.button(1).onTrue(new AimToAngle(60.0, 10.0));
      driver.button(2).onTrue(new AimToAngle(120.0, 5.0));
      driver.button(3).onTrue(new AimToAngle(180.0, 20.0));
      driver.button(4).onTrue(new AimToAngle(330.0, 15.0));
      driver.button(5).onTrue(new AimToAngle(750.0, 15.0));
      */
      driver.button(1).whileTrue(Turret.getInstance().yawQuasistaticForward);
      driver.button(2).whileTrue(Turret.getInstance().yawQuasistaticReverse);
      driver.button(3).whileTrue(Turret.getInstance().yawDynamicForward);
      driver.button(4).whileTrue(Turret.getInstance().yawDynamicReverse);
      
      driver.button(5).whileTrue(Turret.getInstance().pitchQuasistaticForward);
      driver.button(6).whileTrue(Turret.getInstance().pitchQuasistaticReverse);
      driver.button(7).whileTrue(Turret.getInstance().pitchDynamicForward);
      driver.button(8).whileTrue(Turret.getInstance().pitchDynamicReverse);
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
