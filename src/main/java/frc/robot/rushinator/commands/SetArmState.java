package frc.robot.rushinator.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.algaepivot.AlgaeSubsystem;
import frc.robot.rushinator.RushinatorPivot;
import frc.robot.rushinator.RushinatorRollers;
import frc.robot.rushinator.RushinatorWrist;

public class SetArmState extends Command{
    RushinatorPivot mRushinatorPivot;
    RushinatorWrist mRushinatorWrist;
    Rotation2d targetAngle;
    RushinatorPivot.State kTargetState;

    public SetArmState(RushinatorPivot.State targetState) {
        mRushinatorPivot = RushinatorPivot.getInstance();  
        kTargetState = targetState; 
        // targetAngle = targetState.pos;
        addRequirements(mRushinatorPivot);
    }

    // public SetArmState(Supplier<Rotation2d> angleSupplier) {
    //     mRushinatorPivot = RushinatorPivot.getInstance();   
    //     targetAngle = angleSupplier.get();
    //     addRequirements(mRushinatorPivot);
    // }

    @Override
    public void initialize() {
        mRushinatorPivot.setTargetState(kTargetState);
    }

    @Override
    public void end(boolean interrupted) {

    }
}
