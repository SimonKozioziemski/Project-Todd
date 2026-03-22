// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Jumper;

public class RobotContainer {
  public Jumper flyfly = new Jumper();

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    CommandXboxController controller = new CommandXboxController(0);
    flyfly.setDefaultCommand(
        flyfly.drive(controller::getLeftX, controller::getLeftY, controller::getRightX, controller::getRightY));
    controller.a().whileTrue(
        flyfly.jump());
    controller.y().onTrue(flyfly.flyToHub());
    // populateCrowd();
  }

  public void populateCrowd() {
    Pose3d[] robots = new Pose3d[30];
    for (double i = 0; i < 3; i++) {
      for (double x = 0; x < 1; x++) {
        robots[(int) (x + i * 1)] = new Pose3d(x * 10, i * 5, i, new Rotation3d(0, 0, 0));
      }
    }

    DogLog.log("robotS", robots);
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
