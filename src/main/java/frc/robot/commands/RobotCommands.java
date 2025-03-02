package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.algaepivot.AlgaeSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class RobotCommands {
    public static Command coralPrime(CoralSubsystem.State coralState, ElevatorSubsystem.State eleState) {
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                new InstantCommand(() -> ElevatorSubsystem.getInstance().setTargetState(eleState)),
                new InstantCommand(() -> CoralSubsystem.getInstance().setTargetState(coralState))
            )
        );
    }

    public static Command algaePrime(AlgaeSubsystem.State algaeState, ElevatorSubsystem.State eleState) {
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                new InstantCommand(() -> ElevatorSubsystem.getInstance().setTargetState(eleState)),
                new InstantCommand(() -> AlgaeSubsystem.getInstance().setTargetState(algaeState))
            )
        );
    }
}
