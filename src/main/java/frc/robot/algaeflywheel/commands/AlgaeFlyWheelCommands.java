package frc.robot.algaeflywheel.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.Supplier;

import com.ctre.phoenix6.signals.InvertedValue;

public class AlgaeFlyWheelCommands {
    public static Command setAngularVelocity(Supplier<Rotation2d> leftVelocitySupplier, Supplier<Rotation2d> rightVelocitySupplier, InvertedValue kLeftInvertedValue, InvertedValue kRightInvertedValue) {
        return new SetVelocityAlgaeFlyWheel(leftVelocitySupplier, rightVelocitySupplier, kLeftInvertedValue, kRightInvertedValue);
    }

    public static Command setAngularVelocity(Supplier<Rotation2d> velocitySupplier, InvertedValue kLeftInvertedValue, InvertedValue kRightInvertedValue) {
        return setAngularVelocity(velocitySupplier, velocitySupplier, kLeftInvertedValue, kRightInvertedValue);
    }
}