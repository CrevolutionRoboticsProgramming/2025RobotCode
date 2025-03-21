package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.algaepivot.AlgaeSubsystem;
import frc.robot.coralArm.CoralSubsystem;
import frc.robot.elevator.ElevatorSubsystem;
import frc.robot.elevator.commands.SetElevatorState;
import frc.robot.rushinator.RushinatorPivot;
import frc.robot.rushinator.RushinatorWrist;
import frc.robot.rushinator.commands.SetArmState;
import frc.robot.rushinator.commands.SetWristState;

public class RobotCommands {
    public static Command coralPrime(RushinatorPivot.State armState, ElevatorSubsystem.State eleState, RushinatorWrist.State wristState) {
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                new SetElevatorState(eleState),
                new SetArmState(armState),
                new SetWristState(wristState)
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
