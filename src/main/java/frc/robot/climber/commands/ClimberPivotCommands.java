package frc.robot.climber.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.climber.Climber;

public class ClimberPivotCommands {
    public static Command setClimberAngle(Climber.State state) {
        return new SetClimberAngle(state.pos);
    }

    public static Command setClimberAngle(Supplier<Rotation2d> targetSupplier) {
        return new SetClimberAngle(targetSupplier);
    }
}
