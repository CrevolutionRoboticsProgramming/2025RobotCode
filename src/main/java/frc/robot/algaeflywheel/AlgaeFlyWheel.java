// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.algaeflywheel;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
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

        static final InvertedValue kAlgaeScoringInverted = InvertedValue.Clockwise_Positive;
        static final InvertedValue kAlgaeIntakingInverted = InvertedValue.CounterClockwise_Positive;

        static final Slot0Configs kAlgaeFlyWheelConfigs = new Slot0Configs()
            .withKS(0.0)
            .withKV(0.115)
            .withKA(0.1)
            .withKP(0.01)
        ;

        // 5800 RPM at motor; 11600 RPM at wheels
        public static final Rotation2d kMaxAngularVelocity = Rotation2d.fromRotations(6000.0 / 60.0);

        public static final double kCurrentLimit = 60.0;
    }

    private static AlgaeFlyWheel mInstance;
    private final TalonFX LeftFlyWheel, RightFlyWheel;
    private MotorOutputConfigs leftMotorConfigs, rightMotorConfigs;
    private TalonFXConfigurator leftTalonFXConfigurator, rightTalonFXConfigurator;


    private AlgaeFlyWheel() {
        LeftFlyWheel = new TalonFX(Settings.kLeftId, "Canivore");
        var leftTalonFXConfigurator = LeftFlyWheel.getConfigurator();
        leftMotorConfigs = new MotorOutputConfigs();

        leftTalonFXConfigurator.apply(Settings.kAlgaeFlyWheelConfigs);


        RightFlyWheel = new TalonFX(Settings.kRightId, "Canivore");
        var rightTalonFXConfigurator = RightFlyWheel.getConfigurator();
        rightMotorConfigs = new MotorOutputConfigs();

        rightTalonFXConfigurator.apply(Settings.kAlgaeFlyWheelConfigs);
    }

    public static AlgaeFlyWheel getInstance() {
        if (mInstance == null) {
            mInstance = new AlgaeFlyWheel();
        }
        return mInstance;
    }

    public void setLeftFlywheelVelocity(Rotation2d velocity, InvertedValue kInvertedValue) {
        // set invert to CW+ and apply config change
        leftMotorConfigs.Inverted = kInvertedValue;
        leftTalonFXConfigurator.apply(leftMotorConfigs);

        LeftFlyWheel.setControl(new VelocityVoltage(velocity.getRotations()));
    }

    public void setRightFlywheelVelocity(Rotation2d velocity, InvertedValue kInvertedValue) {
        // set invert to CW+ and apply config change
        rightMotorConfigs.Inverted = kInvertedValue;
        rightTalonFXConfigurator.apply(rightMotorConfigs);

        RightFlyWheel.setControl(new VelocityVoltage(velocity.getRotations()));
    }

    public Rotation2d getLeftFlywheelVelocity() {
        return Rotation2d.fromRotations(LeftFlyWheel.getVelocity().getValueAsDouble());
    }

    public Rotation2d getRightFlywheelVelocity() {
        return Rotation2d.fromRotations(RightFlyWheel.getVelocity().getValueAsDouble());
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Left Flywheel Velocity (RPM)", LeftFlyWheel.getVelocity().getValueAsDouble() * 60);
        SmartDashboard.putNumber("Right Flywheel Velocity (RPM)", RightFlyWheel.getVelocity().getValueAsDouble() * 60);
    }
}



