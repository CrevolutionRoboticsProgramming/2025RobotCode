// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.algaeshooterpivot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.algaeshooterpivot.AlgaeShooterPivot;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetAngleAlgaePivot extends Command {
  AlgaeShooterPivot mAlgaeShooterPivot;
    Supplier<Rotation2d> targetSupplier;
    boolean singleShot =  true;

        public enum Preset {
        kZero(Rotation2d.fromDegrees(2)),
        kHandoff(Rotation2d.fromDegrees(0)),
        kHandoffClear(Rotation2d.fromDegrees(5)),
        kShooterNear(Rotation2d.fromDegrees(0)),
        kShooterMid(Rotation2d.fromDegrees(22.25)),
        kShooterFarAuton(Rotation2d.fromDegrees(26)),
        kShooterFar(Rotation2d.fromDegrees(25.5)),
        kTrap(Rotation2d.fromDegrees(153.15)),
        kClimb(Rotation2d.fromDegrees(170)),
        kPass(Rotation2d.fromDegrees(10)),
        kAmp(Rotation2d.fromDegrees(85));

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

