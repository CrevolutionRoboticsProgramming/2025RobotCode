package frc.robot.climber.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.climber.Climber;
import frc.robot.elbowPivot.ElbowPivot;

public class SetClimberAngle extends Command{
    Climber mClimber;
    Supplier<Rotation2d> targetSupplier;

    public enum Preset {
        kZero(Rotation2d.fromDegrees(2)),
        kClimbP2(Rotation2d.fromDegrees(5)),
        kClimbP1(Rotation2d.fromDegrees(0)),
        kStowed(Rotation2d.fromDegrees(170));

        Rotation2d target;

        Preset(Rotation2d target) {
            this.target = target;
        }

        public double getDegrees() {
            return target.getDegrees();
        }

        public Rotation2d getRotation2d() {
            return target;
        }
    }

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
        

    }

    @Override
    public void execute() {
        mClimber.setTargetAngle(targetSupplier.get());
    }


    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
