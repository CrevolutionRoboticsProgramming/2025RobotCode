

package frc.robot.algaepivot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.algaepivot.AlgaeSubsystem;

public class SetAngleAlgaePivot extends Command {
    AlgaeSubsystem mAlgaeSubsystem;
    Rotation2d targetAngle;
    AlgaeSubsystem.State mTargetState;

    public SetAngleAlgaePivot(AlgaeSubsystem.State targetState) {
        mAlgaeSubsystem = AlgaeSubsystem.getInstance(); 
        mTargetState = targetState;  
        targetAngle = targetState.pos;
        addRequirements(mAlgaeSubsystem);
    }

    public SetAngleAlgaePivot(Supplier<Rotation2d> angleSupplier) {
        mAlgaeSubsystem = AlgaeSubsystem.getInstance();   
        targetAngle = angleSupplier.get();
        addRequirements(mAlgaeSubsystem);
    }

    @Override
    public void initialize() {
        mAlgaeSubsystem.setTargetState(mTargetState);

    }

    @Override
    public void end(boolean interrupted) {

    }
}

