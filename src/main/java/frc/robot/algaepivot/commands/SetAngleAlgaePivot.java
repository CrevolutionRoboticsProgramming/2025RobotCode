

package frc.robot.algaepivot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.algaepivot.AlgaeShooterPivot;
import frc.robot.subsystems.AlgaeSubsystem;

public class SetAngleAlgaePivot extends Command {
    AlgaeSubsystem mAlgaeSubsystem;
    AlgaeSubsystem.State targetState;

    // public enum State {
    //     kFloorIntake(Rotation2d.fromRotations(-0.04)),
    //     kReefIntake(Rotation2d.fromRotations(0)),
    //     kScore(Rotation2d.fromRotations(0.15)),
    //     kStow(Rotation2d.fromRotations(0.23));

    //     State(Rotation2d pos) {
    //         this.pos = pos;
    //     }
    //     public final Rotation2d pos;
    

    //     public double getDegrees() {
    //         return pos.getDegrees();
    //     }

    //     public Rotation2d getRotation2d() {
    //         return pos;
    //     }
    // }

    public SetAngleAlgaePivot(AlgaeSubsystem.State targetState) {
        mAlgaeSubsystem = AlgaeSubsystem.getInstance();   
        this.targetState = targetState;
        addRequirements(mAlgaeSubsystem);
    }

    @Override
    public void initialize() {
        mAlgaeSubsystem.setTargetPosition(targetState.pos);

    }

    @Override
    public void end(boolean interrupted) {

    }
}

