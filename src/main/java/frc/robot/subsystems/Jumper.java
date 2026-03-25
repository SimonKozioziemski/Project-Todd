package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotation;

import java.util.function.DoubleSupplier;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FuelSim;

public class Jumper extends SubsystemBase {
    private Pose3d pose = new Pose3d(3,5,0, Rotation3d.kZero);
    public final double gravity = -0.1;
    private double vz = 0;
    private double vx = 0;
    private double vy = 0;
    private Translation3d position = new Translation3d();
    private Rotation3d rotation = new Rotation3d();

    public Command drive(DoubleSupplier x, DoubleSupplier y, DoubleSupplier rot, DoubleSupplier pitch) {
        return Commands.run(() -> {
            // position = position.plus(new Translation3d(-y.getAsDouble() * 0.01,
            // -x.getAsDouble() * 0.01, 0));
            // rotation = rotation.plus(new Rotation3d(0,-pitch.getAsDouble() * 0.1,
            // -rot.getAsDouble() * 0.005));
            pose = pose.plus(new Transform3d(-MathUtil.applyDeadband(y.getAsDouble(), 0.15) * 0.1,
                    -MathUtil.applyDeadband(x.getAsDouble(), 0.15) * 0.1, 0,
                    new Rotation3d(0, -MathUtil.applyDeadband(pitch.getAsDouble(), 0.15) * 0.1,
                            -MathUtil.applyDeadband(rot.getAsDouble(), 0.15) * 0.1)));
        }, this);
    }

    public double applyDeadband(double input, double threshold) {
        if (Math.abs(input) < threshold) {
            return 0.0;
        }
        return input;
    }

    @Override
    public void periodic() {
        position = pose.getTranslation();
        if (position.getZ() < 0) {
            vz = 0;
            position = new Translation3d(position.getX(), position.getY(), 0);
        } else if (position.getZ() > 1) {
            vz += gravity;
        }

        if(position.getZ() <= 0){
                rotation = new Rotation3d(0,0, pose.getRotation().getZ());
        }
        else{
            rotation = pose.getRotation();
        }
        position = position.plus(new Translation3d(0, 0, vz * 0.02));
        // position = pose.getTranslation();
        pose = new Pose3d(position, rotation);
        DogLog.log("Pose", pose);
    }

    public Command jump() {
        return new RunCommand(() -> this.vz = 3);
    }

    public Command flyToHub() {
        return new InstantCommand(() -> this.pose = new Pose3d(4.6, 4.0, 2.5, Rotation3d.kZero));
    }

    public Command dash(){
        return Commands.startRun(() -> this.vz = 3, () -> {
            pose = pose.plus(new Transform3d(0.01, 0, 0, new Rotation3d(0, 0.1, 0)));
        }, this).withTimeout(2.5);
    }

    public Pose3d whereIsFlyFly() {
        return pose;
    }

    public Command shoot(){
        return Commands.repeatingSequence(
            new InstantCommand(() -> launchFuel()),
            Commands.waitSeconds(0.1)
        );
    }

    public void launchFuel() {
        Translation3d initialPosition = this.whereIsFlyFly().getTranslation();
            // initialPosition = new Translation3d(5,5, 5);
        double vx = this.whereIsFlyFly().getRotation().toRotation2d().getCos();
        double vy = this.whereIsFlyFly().getRotation().toRotation2d().getSin();
        double ball_vz = Math.cos(this.whereIsFlyFly().getRotation().getZ()) * 8;
        Translation3d velocity = new Translation3d(vx, vy, ball_vz);
        FuelSim.getInstance().spawnFuel(initialPosition, velocity);
    }

}