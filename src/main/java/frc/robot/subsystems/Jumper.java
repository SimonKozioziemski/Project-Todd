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

public class Jumper extends SubsystemBase {
    private Pose3d pose = new Pose3d();
    public final double gravity = -0.1;
    private double vz = 0;
    // private double vx = 0;
    // private double vy = 0;
    private Translation3d position = new Translation3d();
    private Rotation3d rotation = new Rotation3d();

    public Command drive(DoubleSupplier x, DoubleSupplier y, DoubleSupplier rot, DoubleSupplier pitch) {
        return Commands.run(() -> {
            // position = position.plus(new Translation3d(-y.getAsDouble() * 0.01,
            // -x.getAsDouble() * 0.01, 0));
            // rotation = rotation.plus(new Rotation3d(0,-pitch.getAsDouble() * 0.1,
            // -rot.getAsDouble() * 0.005));
            pose = pose.plus(new Transform3d(2 * -MathUtil.applyDeadband(y.getAsDouble(), 0.15) * 0.1, 2 * -MathUtil.applyDeadband(x.getAsDouble(), 0.15) * 0.1, 0,
                    new Rotation3d(0, MathUtil.applyDeadband(pitch.getAsDouble(), 0.15) * 0.01, MathUtil.applyDeadband(rot.getAsDouble(), 0.15) * 0.1)));
        }, this);
    }

    public double applyDeadband(double input, double threshold){
        if(Math.abs(input) < threshold){
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
            // rotation = new Rotation3d(rotation.getX(), pose.getRotation(), rotation.getZ());
        } else if (position.getZ() > 1) {
            vz += gravity;
        }
        position = position.plus(new Translation3d(0, 0, vz * 0.02));
        // position = pose.getTranslation();
        pose = new Pose3d(position, pose.getRotation());
        DogLog.log("Pose", pose);
    }

    public Command jump() {
        return new RunCommand(() -> this.vz = 3);
    }

    public Command flyToHub(){
        return new InstantCommand(() -> this.pose = new Pose3d(4.6, 4.0, 2.5, Rotation3d.kZero));
    }

    public Pose3d whereIsFlyFly(){
        return pose;
    }

}