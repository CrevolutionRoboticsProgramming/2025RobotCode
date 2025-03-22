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

    public static Command coralPrimeShoot(RushinatorPivot.State armState, RushinatorWrist.State wristState) {
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                new SetArmState(armState),
                new SetWristState(wristState)
            )
        );
    }

    public static Command toggleWristState() {
        System.out.println("Last State in Toggle Wrist" + RushinatorWrist.kLastState.name());
        if(RushinatorWrist.kLastState == State.kScoreLeftWrist) {
            System.out.println("First If");
            return new SetWristState(RushinatorWrist.State.kScoreRightWrist);
        }
        else if(RushinatorWrist.kLastState == State.kScoreRightWrist) {
            System.out.println("2nd If");
            return new SetWristState(RushinatorWrist.State.kScoreLeftWrist);
        }
        else if(RushinatorWrist.kLastState == State.kTravelLeft) {
            System.out.println("3rd If");
            return new SetWristState(RushinatorWrist.State.kTravelRight);
        }
        else if(RushinatorWrist.kLastState == State.kTravelRight) {
            System.out.println("Fourth If");
            return new SetWristState(RushinatorWrist.State.kTravelLeft);
        }
        else {
            System.out.println("Else");
            return new SetWristState(RushinatorWrist.kLastState);
        }
        // if(RushinatorPivot.kLastState == RushinatorPivot.State.kScore) {
        //     RushinatorWrist.State toggledWristState = (RushinatorWrist.kLastState == State.kScoreLeftWrist) ? State.kScoreRightWrist : State.kScoreLeftWrist;
        //     return new SetWristState(toggledWristState);
        // }
        // else if(RushinatorPivot.kLastState == RushinatorPivot.State.kStowTravel) {
        //     RushinatorWrist.State toggledWristState = (RushinatorWrist.kLastState == State.kTravelLeft) ? State.kTravelRight : State.kTravelLeft;
        //     return new SetWristState(toggledWristState);
        // }
        // else {
        //     return new SetWristState(RushinatorWrist.kLastState);
        // }
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
