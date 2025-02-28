

package frc.robot.algaepivot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.algaepivot.AlgaeSubsystem;

public class SetAngleAlgaePivot extends Command {
    AlgaeSubsystem mAlgaeSubsystem;
    AlgaeSubsystem.State targetState;
    Rotation2d targetAngle;

    public SetAngleAlgaePivot(AlgaeSubsystem.State targetState) {
        mAlgaeSubsystem = AlgaeSubsystem.getInstance();   
        this.targetState = targetState;
        addRequirements(mAlgaeSubsystem);
    }

    // public SetAngleAlgaePivot(Supplier<Rotation2d> angleSupplier) {
    //     mAlgaeSubsystem = AlgaeSubsystem.getInstance();   
    //     targetAngle = angle;
    //     addRequirements(mAlgaeSubsystem);
    // }

    @Override
    public void initialize() {
        mAlgaeSubsystem.setTargetPosition(targetState.pos);

    }

    @Override
    public void end(boolean interrupted) {

    }
}

