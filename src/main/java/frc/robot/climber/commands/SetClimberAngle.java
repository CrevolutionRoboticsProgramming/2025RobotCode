package frc.robot.climber.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.climber.Climber;

public class SetClimberAngle extends Command{
    Climber mClimber;
    Supplier<Rotation2d> targetSupplier;

    public SetClimberAngle(Rotation2d angleIn) {
        mClimber = Climber.getInstance();   
        targetSupplier = ()-> angleIn;
    }
    public SetClimberAngle(Supplier<Rotation2d> angleIn) {
        mClimber = Climber.getInstance();  
        targetSupplier = angleIn; 
    }

    @Override
    public void initialize() {
        mClimber.setTargetPos(targetSupplier.get());
    }

    @Override
    public void execute() {
        mClimber.setTargetPos(targetSupplier.get());
    }


    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
