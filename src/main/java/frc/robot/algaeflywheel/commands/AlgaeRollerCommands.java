package frc.robot.algaeflywheel.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class AlgaeRollerCommands {
    public static Command autoShootAlgae(Supplier<Rotation2d> velocitySupplier) {
        return new SetRollerVelAutoShoot(velocitySupplier);
    }
}
