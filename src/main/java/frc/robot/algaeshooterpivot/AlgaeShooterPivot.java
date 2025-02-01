// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.algaeshooterpivot;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public class AlgaeShooterPivot extends SubsystemBase{
    public class Settings{
        public static int kTalonID = 19;
        public static boolean isInverted = false;

        static final Slot0Configs kAlgaeShooterPivotConfigs = new Slot0Configs()
            .withKS(0.0)
            .withKV(0.115)
            .withKP(0.0)
            .withKI(0.0)
            .withKD(0.0);
    }
    private static AlgaeShooterPivot mInstance;
    private TalonFX mKraken;

    public AlgaeShooterPivot() {
        mKraken = new TalonFX(Settings.kTalonID);
    
        var talonFXConfigurator = mKraken.getConfigurator();
        var motorConfigs = new MotorOutputConfigs();

        motorConfigs.Inverted = InvertedValue.Clockwise_Positive;
        talonFXConfigurator.apply(motorConfigs);

        mKraken.getConfigurator().apply(Settings.kAlgaeShooterPivotConfigs);
        
        // Trapezoid profile with max velocity 80 rps, max accel 160 rps/s
        final TrapezoidProfile m_profile = new TrapezoidProfile(
        new TrapezoidProfile.Constraints(80, 160)
        );
        // Final target of 200 rot, 0 rps
        TrapezoidProfile.State m_goal = new TrapezoidProfile.State(200, 0);
        TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();

        // create a position closed-loop request, voltage output, slot 0 configs
        final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);

        // calculate the next profile setpoint
        m_setpoint = m_profile.calculate(0.020, m_setpoint, m_goal);

        // send the request to the device
        m_request.Position = m_setpoint.position;
        m_request.Velocity = m_setpoint.velocity;
        mKraken.setControl(m_request);
    }
    public static AlgaeShooterPivot getInstance() {
        if (mInstance == null) {
            mInstance = new AlgaeShooterPivot();
        }
        return mInstance;
    }
    public void setPivotVelocity(Rotation2d velocity) {
        mKraken.setControl(new VelocityVoltage(velocity.getRotations()));
    }
    public Rotation2d getPivotVelocity() {
        return Rotation2d.fromRotations(mKraken.getVelocity().getValueAsDouble());
    }
}
