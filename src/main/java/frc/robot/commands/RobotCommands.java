package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.algaepivot.AlgaeSubsystem;
import frc.robot.coralArm.CoralSubsystem;
import frc.robot.elevator.ElevatorSubsystem;
import frc.robot.elevator.commands.SetElevatorState;
import frc.robot.rushinator.RushinatorPivot;
import frc.robot.rushinator.RushinatorWrist;
import frc.robot.rushinator.RushinatorWrist.State;
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

    public static Command toggleWristState() {
        if(RushinatorPivot.kLastState == RushinatorPivot.State.kScore) {
            RushinatorWrist.State toggledWristState = (RushinatorWrist.kLastState == State.kScoreLeftWrist) ? State.kScoreRightWrist : State.kScoreLeftWrist;
            return coralPrime(RushinatorPivot.kLastState, ElevatorSubsystem.kLastState, toggledWristState);
        }
        else if(RushinatorPivot.kLastState == RushinatorPivot.State.kStowTravel) {
            RushinatorWrist.State toggledWristState = (RushinatorWrist.kLastState == State.kTravelLeft) ? State.kTravelRight : State.kTravelLeft;
            return coralPrime(RushinatorPivot.kLastState, ElevatorSubsystem.kLastState, toggledWristState);
        }
        else {
            return coralPrime(RushinatorPivot.kLastState, ElevatorSubsystem.kLastState, RushinatorWrist.kLastState);
        }
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
