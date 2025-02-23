package frc.robot.elbowpivot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class ElbowPivotCommands {
    public static Command setElbowPivotAngle(SetElbowPivotAngle.Preset state) {
        return new SetElbowPivotAngle(state.getRotation2d());
    }

    public static Command setElbowPivotAngle(Supplier<Rotation2d> targetSupplier) {
        return new SetElbowPivotAngle(targetSupplier);
    }
}
