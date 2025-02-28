package frc.robot.algaepivot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.algaepivot.AlgaeSubsystem;

public class AlgaePivotCommands {
    public static Command setAlgaePivotAngle(AlgaeSubsystem.State state) {
        return new SetAngleAlgaePivot(state);
    }

    // public static Command setAlgaePivotAngle(Supplier<Rotation2d> velocitySupplier, boolean openLoop) {
    //     return new SetAngleAlgaePivot(velocitySupplier, openLoop);
    // }
    
}
