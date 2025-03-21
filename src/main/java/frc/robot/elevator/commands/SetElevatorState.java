package frc.robot.elevator.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.elevator.ElevatorSubsystem;
import frc.robot.rushinator.RushinatorPivot;


public class SetElevatorState extends Command{
    ElevatorSubsystem mElevator;
    ElevatorSubsystem.State kTargetState;
    
    public SetElevatorState(ElevatorSubsystem.State targetState) {
        mElevator = ElevatorSubsystem.getInstance();
        kTargetState = targetState; 
        // targetAngle = targetState.pos;
        addRequirements(mElevator);
    }

    @Override
    public void initialize() {
        mElevator.setTargetState(kTargetState);
    }

    @Override
    public void end(boolean interrupted) {

    }
}
