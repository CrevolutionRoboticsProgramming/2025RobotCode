package frc.robot.algaepivot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class AlgaePivotCommands {
    public static Command setAlgaePivotAngle(Supplier<Rotation2d> targetSupplier) {
        return new SetAngleAlgaePivot(targetSupplier);
    }

    public static Command setAlgaePivotAngle(SetAngleAlgaePivot.Preset state) {
        return new SetAngleAlgaePivot(state.getRotation2d());
    }

    public static Command setAlgaePivotAngle(Rotation2d angle) {
        return new SetAngleAlgaePivot(angle);
    }

    public static Command setAlgaePivotAngle(Supplier<Rotation2d> velocitySupplier, boolean openLoop) {
        return new SetAngleAlgaePivot(velocitySupplier, openLoop);
    }
}
