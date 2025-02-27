package frc.robot.elevator.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ElbowPivot.ElbowPivot;
import frc.robot.elevator.Elevator;

public class SetElevatorPosition extends Command{
    Elevator mElevator;
    Supplier<Rotation2d> targetSupplier;

    public enum Preset {
        kZero(Rotation2d.fromDegrees(2)),
        kCoralL1(Rotation2d.fromDegrees(0)),
        kCoralL2(Rotation2d.fromDegrees(5)),
        kCoralL3(Rotation2d.fromDegrees(0)),
        kCoralL4(Rotation2d.fromDegrees(22.25)),
        kAlgaeL2(Rotation2d.fromDegrees(26)),
        kAlgaeL3(Rotation2d.fromDegrees(25.5)),
        kAlgaeShoot(Rotation2d.fromDegrees(153.15)),
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

    public SetElevatorPosition(Rotation2d angleIn) {
        mElevator = Elevator.getInstance();   
        targetSupplier = ()-> angleIn;
    }
    public SetElevatorPosition(Supplier<Rotation2d> angleIn) {
        mElevator = Elevator.getInstance();  
        targetSupplier = angleIn; 
    }

    @Override
    public void initialize() {
        

    }

    @Override
    public void execute() {
        mElevator.setTargetPosition(targetSupplier.get());
    }


    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
