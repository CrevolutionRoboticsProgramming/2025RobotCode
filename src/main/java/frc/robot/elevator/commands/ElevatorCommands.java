package frc.robot.elevator.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ElbowPivot.commands.SetElbowPivotAngle;

public class ElevatorCommands {
    public static Command setElevatorPostion(SetElbowPivotAngle.Preset state) {
        return new SetElevatorPosition(state.getRotation2d());
    }

    public static Command setElevatorPostion(Supplier<Rotation2d> targetSupplier) {
        return new SetElevatorPosition(targetSupplier);
    }
}
