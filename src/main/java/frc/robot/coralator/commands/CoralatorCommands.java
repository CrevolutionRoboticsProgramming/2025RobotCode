package frc.robot.coralator.commands;

import java.util.function.Supplier;

import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class CoralatorCommands {
    public Command setVelocityCoral(double voltage) {
        return new SetVelocityCoralator(voltage);
    }

    
}
