// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.algaeflywheel;

import org.opencv.core.RotatedRect;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

/** Add your docs here. */
public class AlgaeRoller extends SubsystemBase{
    public static class Settings {
        static final int kLeftID = 15;
        static final int kRightID = 16;
        static final int kIndexID = 17;

        static final double kCurrentLimit = 25.0;

        static final Rotation2d kMaxAngluarVelocity = Rotation2d.fromRotations(6000 / 60);
    }

    private static AlgaeRoller mInstance;
    private final TalonFX mTalonShooterLeft, mTalonShooterRight, mTalonIndexer;

    private AlgaeRoller() {
        mTalonShooterLeft = new TalonFX(Settings.kLeftID);
        mTalonShooterRight = new TalonFX(Settings.kRightID);
        mTalonIndexer = new TalonFX(Settings.kIndexID);

        mTalonShooterLeft.getConfigurator().apply(new CurrentLimitsConfigs().withSupplyCurrentLimit(Settings.kCurrentLimit));
        mTalonShooterRight.getConfigurator().apply(new CurrentLimitsConfigs().withSupplyCurrentLimit(Settings.kCurrentLimit));
        mTalonIndexer.getConfigurator().apply(new CurrentLimitsConfigs().withSupplyCurrentLimit(Settings.kCurrentLimit));
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

    public void setFlywheelVelocity(Rotation2d velocity) {
        mTalonShooterLeft.setControl(new VelocityVoltage(velocity.getRotations()));
        mTalonShooterRight.setControl(new VelocityVoltage(velocity.getRotations()));
    }

    public void setIndexerVoltage(double voltage) {
        mTalonIndexer.setVoltage(voltage);
    }

    public Rotation2d geLeftVelocity() {
        return Rotation2d.fromRotations(mTalonShooterLeft.getVelocity().getValueAsDouble());
    }

    public Rotation2d getRightVelocity() {
        return Rotation2d.fromRotations(mTalonShooterRight.getVelocity().getValueAsDouble());
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
            subsystem.setIndexerVoltage(-3);
        }
    }

    public static class SetIndexerVoltagCommand extends Command {
        AlgaeRoller roller;
        double voltage;
        public SetIndexerVoltagCommand(AlgaeRoller subsystem, double voltage) {
            roller = AlgaeRoller.getInstance();
            addRequirements(roller);
            this.voltage = voltage;
        }

        @Override
        public void initialize() {
            roller.setIndexerVoltage(voltage);
            roller.setFlywheelVoltage(0);
        }
    }

    public static class SetFlywheelVoltagCommand extends Command {
        AlgaeRoller roller;
        double voltage;
        public SetFlywheelVoltagCommand(AlgaeRoller subsystem, double voltage) {
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
            roller.setIndexerVoltage(-6);
            roller.setFlywheelVoltage(3);
        }
    }

    public static class PrimeShootCommand extends Command {
        AlgaeRoller roller;
        public PrimeShootCommand() {
            roller = AlgaeRoller.getInstance();
            addRequirements(roller);
        }

        @Override
        public void initialize() {
            roller.setIndexerVoltage(-10);
            roller.setFlywheelVoltage(9.0);
        }
    }

    public static class FarShootCommand extends Command {
        AlgaeRoller roller;
        public FarShootCommand() {
            roller = AlgaeRoller.getInstance();
            addRequirements(roller);
        }

        @Override
        public void initialize() {
            roller.setIndexerVoltage(9.0);
            roller.setFlywheelVoltage(9.0);
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
            roller.setFlywheelVoltage(3);
            roller.setIndexerVoltage(3);
        }
    }

    public static class ProcessShootCommand extends Command {
        AlgaeRoller roller;
        public ProcessShootCommand() {
            roller = AlgaeRoller.getInstance();
            addRequirements(roller);
        }

        @Override
        public void initialize() {
            roller.setFlywheelVoltage(0.5);
            roller.setIndexerVoltage(0.5);
        }
    }
}



