// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Jumper;

public class RobotContainer {
  public Jumper flyfly = new Jumper();
  CommandXboxController controller = new CommandXboxController(0);

  public RobotContainer() {
    configureBindings();
    configureFuelSim();
  }

  private void configureBindings() {
    flyfly.setDefaultCommand(
        flyfly.drive(controller::getLeftX, controller::getLeftY, controller::getRightX, controller::getRightY));
    controller.a().whileTrue(
        flyfly.jump());
    controller.y().onTrue(flyfly.flyToHub());
    controller.rightTrigger().whileTrue(
      flyfly.shoot()
        // new InstantCommand(() -> launchFuel())
    );

    controller.povUp().onTrue(flyfly.dash());
    // populateCrowd();
    // controller.
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

  public void configureFuelSim() {
    FuelSim instance = FuelSim.getInstance();
    // instance.spawnStartingFuel();
    // Register a robot for collision with fuel
    FuelSim.getInstance().registerRobot(
        0.902, // from left to right
        0.902, // from front to back
        0.191, // from floor to top of bumpers
        () -> flyfly.whereIsFlyFly(), // Supplier<Pose2d> of robot pose
        () -> new ChassisSpeeds(0, 0, 0)); // Supplier<ChassisSpeeds> of field-centric chassis speeds
    FuelSim.getInstance().registerIntake(
        -0.4502, -0.635, -0.34925, 0.34925, // robot-centric coordinates for bounding box
        controller.b()); // (optional) BooleanSupplier for whether the intake should be active at
    // a given moment
    instance.start();
    SmartDashboard.putData(Commands.runOnce(() -> {
      FuelSim.getInstance().clearFuel();
      // FuelSim.getInstance().spawnStartingFuel();
    })
        .withName("Reset Fuel")
        .ignoringDisable(true));

  }

  
}
