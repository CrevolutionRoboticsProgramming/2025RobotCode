package frc.robot.coralator.commands;

import java.util.function.Supplier;

import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class CoralatorCommands {
    public static Command setVelocityCoralator(Supplier<Rotation2d> velocitySupplier, InvertedValue kInvertedValue) {
        return new SetVelocityCoralator(velocitySupplier, kInvertedValue);
    }
    
}
