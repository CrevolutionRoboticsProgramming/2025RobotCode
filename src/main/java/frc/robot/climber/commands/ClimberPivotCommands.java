package frc.robot.climber.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class ClimberPivotCommands {
    public static Command setClimberAngle(SetClimberAngle.Preset state) {
        return new SetClimberAngle(state.getRotation2d());
    }

    public static Command setClimberAngle(Supplier<Rotation2d> targetSupplier) {
        return new SetClimberAngle(targetSupplier);
    }
}
