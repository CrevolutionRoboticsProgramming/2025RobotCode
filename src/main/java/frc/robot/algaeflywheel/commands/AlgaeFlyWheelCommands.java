package frc.robot.algaeflywheel.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.Supplier;

public class AlgaeFlyWheelCommands {
    public static Command setAngularVelocity(Supplier<Rotation2d> leftVelocitySupplier, Supplier<Rotation2d> rightVelocitySupplier) {
        return new SetVelocityAlgaeFlyWheel(leftVelocitySupplier, rightVelocitySupplier);
    }

    public static Command setAngularVelocity(Supplier<Rotation2d> velocitySupplier) {
        return setAngularVelocity(velocitySupplier, velocitySupplier);
    }
}
