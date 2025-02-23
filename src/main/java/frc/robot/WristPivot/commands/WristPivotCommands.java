package frc.robot.WristPivot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.elbowpivot.commands.SetElbowPivotAngle;

public class WristPivotCommands {
    public static Command setWristPivotAngle(SetElbowPivotAngle.Preset state) {
        return new SetWristPivotAngle(state.getRotation2d());
    }

    public static Command setWristPivotAngle(Supplier<Rotation2d> targetSupplier) {
        return new SetWristPivotAngle(targetSupplier);
    }
}
