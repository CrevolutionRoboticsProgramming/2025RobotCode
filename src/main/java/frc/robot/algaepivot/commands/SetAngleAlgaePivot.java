

package frc.robot.algaepivot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.algaepivot.AlgaeShooterPivot;

public class SetAngleAlgaePivot extends Command {
    AlgaeShooterPivot mAlgaeShooterPivot;
    Supplier<Rotation2d> targetSupplier;
    boolean singleShot =  true;

        public enum Preset {
        kZero(Rotation2d.fromDegrees(2)),
        kStowed(Rotation2d.fromDegrees(0)),
        kElveatorHigh(Rotation2d.fromDegrees(0)),
        kCoral(Rotation2d.fromDegrees(0)),
        kShoot(Rotation2d.fromDegrees(0));

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

    public SetAngleAlgaePivot(Rotation2d angleIn) {
        mAlgaeShooterPivot = AlgaeShooterPivot.getInstance();   
        targetSupplier = ()-> angleIn;
    }
    public SetAngleAlgaePivot(Supplier<Rotation2d> angleIn) {
        mAlgaeShooterPivot = AlgaeShooterPivot.getInstance();  
        targetSupplier = angleIn; 
    }

    public SetAngleAlgaePivot(Supplier<Rotation2d> angleIn, boolean singleSet) {
        mAlgaeShooterPivot = AlgaeShooterPivot.getInstance();  
        targetSupplier = angleIn;
        singleShot = singleSet; 
    }

    @Override
    public void initialize() {
        

    }

    @Override
    public void execute() {
        mAlgaeShooterPivot.setTargetAngle(targetSupplier.get());
    }


    @Override
    public boolean isFinished() {
        return singleShot;
    }

    @Override
    public void end(boolean interrupted) {

    }
}

