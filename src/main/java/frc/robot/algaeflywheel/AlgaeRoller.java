// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.algaeflywheel;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public class AlgaeRoller extends SubsystemBase{
     public static class Settings {
        static final int kLeftID = 15;
        static final int kRightID = 16;
        static final int kIndexID = 17;
    }

    private static AlgaeRoller mInstance;
    private final TalonFX mTalonShooterLeft, mTalonShooterRight, mTalonIndexer;

    private AlgaeRoller() {
        mTalonShooterLeft = new TalonFX(Settings.kLeftID);
        mTalonShooterRight = new TalonFX(Settings.kRightID);
        mTalonIndexer = new TalonFX(Settings.kIndexID);
    }

    public static AlgaeRoller getInstance() {
        if (mInstance == null) {
            mInstance = new AlgaeRoller();
        }
        return mInstance;
    }

    public void setFlywheelVoltage(double voltage) {
        mTalonShooterLeft.setVoltage(voltage);
        mTalonShooterRight.setVoltage(voltage);
    }

    public void setIndexerVoltage(double voltage) {
        mTalonIndexer.setVoltage(voltage);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Left Flywheel Velocity (RPM)", mTalonShooterLeft.getVelocity().getValueAsDouble() * 60);
        SmartDashboard.putNumber("Right Flywheel Velocity (RPM)", mTalonShooterRight.getVelocity().getValueAsDouble() * 60);
    }

    public static class DefaultCommand extends Command {
        AlgaeRoller subsystem;
        public DefaultCommand() {
            this.subsystem = AlgaeRoller.getInstance();
            addRequirements(subsystem);
        }

        @Override
        public void initialize() {
            subsystem.setFlywheelVoltage(0);
            subsystem.setIndexerVoltage(0);
        }
    }

    public static class SetVoltageCommand extends Command {
        AlgaeRoller roller;
        double voltage;
        public SetVoltageCommand(AlgaeRoller subsystem, double voltage) {
            roller = AlgaeRoller.getInstance();
            addRequirements(roller);
            this.voltage = voltage;
        }

        @Override
        public void initialize() {
            roller.setFlywheelVoltage(voltage);
        }
    }

    public static class IntakeCommand extends Command {
        AlgaeRoller roller;
        public IntakeCommand() {
            roller = AlgaeRoller.getInstance();
            addRequirements(roller);
        }

        @Override
        public void initialize() {
            roller.setFlywheelVoltage(-6);
            roller.setIndexerVoltage(-6);
        }
    }

    public static class PrimeCommand extends Command {
        AlgaeRoller roller;
        public PrimeCommand() {
            roller = AlgaeRoller.getInstance();
            addRequirements(roller);
        }

        @Override
        public void initialize() {
            roller.setFlywheelVoltage(12);
            roller.setIndexerVoltage(-6);
        }
    }

    public static class ShootCommand extends Command {
        AlgaeRoller roller;
        public ShootCommand() {
            roller = AlgaeRoller.getInstance();
            addRequirements(roller);
        }

        @Override
        public void initialize() {
            roller.setFlywheelVoltage(12);
            roller.setIndexerVoltage(12);
        }
    }
}



