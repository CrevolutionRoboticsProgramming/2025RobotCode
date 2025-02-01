// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.algaeflywheel;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public class AlgaeFlyWheel extends SubsystemBase{
     public static class Settings {
        static final int kLeftId = 27;
        static final int kRightId = 28;

        static final Slot0Configs kFlywheelConfigs = new Slot0Configs()
                .withKS(0.0)
                .withKV(0.115)
                .withKP(0.0);

        // 5800 RPM at motor; 11600 RPM at wheels
        public static final Rotation2d kMaxAngularVelocity = Rotation2d.fromRotations(6000.0 / 60.0);

        public static final double kCurrentLimit = 60.0;
    }

    private static AlgaeFlyWheel mInstance;
    private final TalonFX mKrakenLeft, mKrakenRight;

    private AlgaeFlyWheel() {
        mKrakenLeft = new TalonFX(Settings.kLeftId);
        var leftTalonFXConfigurator = mKrakenLeft.getConfigurator();
        var leftMotorConfigs = new MotorOutputConfigs();

        leftMotorConfigs.Inverted = InvertedValue.Clockwise_Positive;
        leftTalonFXConfigurator.apply(leftMotorConfigs);


        mKrakenRight = new TalonFX(Settings.kRightId);
        var rightTalonFXConfigurator = mKrakenRight.getConfigurator();
        var righttMotorConfigs = new MotorOutputConfigs();

        righttMotorConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
        rightTalonFXConfigurator.apply(righttMotorConfigs);
    }

    // 57 degreedf

    public static AlgaeFlyWheel getInstance() {
        if (mInstance == null) {
            mInstance = new AlgaeFlyWheel();
        }
        return mInstance;
    }

    public void setLeftFlywheelVelocity(Rotation2d velocity) {
        mKrakenLeft.setControl(new VelocityVoltage(velocity.getRotations()));
    }

    public void setRightFlywheelVelocity(Rotation2d velocity) {
        mKrakenRight.setControl(new VelocityVoltage(velocity.getRotations()));
    }

    public Rotation2d getLeftFlywheelVelocity() {
        return Rotation2d.fromRotations(mKrakenLeft.getVelocity().getValueAsDouble());
    }

    public Rotation2d getRightFlywheelVelocity() {
        return Rotation2d.fromRotations(mKrakenRight.getVelocity().getValueAsDouble());
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Left Flywheel Velocity (RPM)", mKrakenLeft.getVelocity().getValueAsDouble() * 60);
        SmartDashboard.putNumber("Right Flywheel Velocity (RPM)", mKrakenRight.getVelocity().getValueAsDouble() * 60);
    }
}



