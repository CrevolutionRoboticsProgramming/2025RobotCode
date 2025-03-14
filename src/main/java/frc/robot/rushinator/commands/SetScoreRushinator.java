package frc.robot.rushinator.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.algaepivot.AlgaeSubsystem;
import frc.robot.rushinator.RushinatorPivot;
import frc.robot.rushinator.RushinatorRollers;
import frc.robot.rushinator.RushinatorWrist;

public class SetScoreRushinator extends Command{
    RushinatorPivot mRushinatorPivot;
    RushinatorRollers mRushinatorRollers;
    RushinatorWrist mRushinatorWrist;
    Rotation2d targetAngle;

    public SetScoreRushinator(RushinatorPivot.State targetState) {
        mRushinatorPivot = RushinatorPivot.getInstance();   
        mRushinatorRollers = RushinatorRollers.getInstance();
        targetAngle = targetState.pos;
        addRequirements(getRequirements());
    }

    public SetScoreRushinator(Supplier<Rotation2d> angleSupplier) {
        mRushinatorPivot = RushinatorPivot.getInstance();   
        mRushinatorRollers = RushinatorRollers.getInstance();
        targetAngle = angleSupplier.get();
        addRequirements(getRequirements());
    }

    @Override
    public void initialize() {
        mRushinatorPivot.setTargetPosition(targetAngle);;

    }

    @Override
    public void end(boolean interrupted) {

    }
}
