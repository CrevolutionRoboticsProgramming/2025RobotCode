// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.algaepivot;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public class AlgaeShooterPivot extends SubsystemBase{
    public class Settings{
        public static int kAlgaePivotTalonID = 19;
        public static int kAlgaePivotEncoder = 1;
        public static boolean isInverted = false;

        static final Slot0Configs kAlgaePivotConfigs = new Slot0Configs()
            .withKG(0.0)
            .withKS(0.0)
            .withKV(0.0)
            .withKA(0.0)
            .withKP(0.0)
            .withKI(0.0)
            .withKD(0.0);

        public static final Rotation2d kMaxAngularVelocity = Rotation2d.fromDegrees(200); //120
        public static final Rotation2d kMaxAngularAcceleration = Rotation2d.fromDegrees(300);
        public static final Rotation2d kMaxAngle = Rotation2d.fromDegrees(180);
        public static final Rotation2d kMaxAnglePhysical = Rotation2d.fromDegrees(175);
    
        public static final int KMaxVoltage = 30;

        // kFFAngleOffset is the differnce between our zero point (handoff) and 'true' zero (parallel to the ground)
        private static final Rotation2d kFFAngleOffset = Rotation2d.fromDegrees(30);
    }
    private static AlgaeShooterPivot mInstance;
    private final CANcoder mEncoder;
    private TalonFX mKraken;
    private Constraints mConstraints;
    private final ProfiledPIDController mPPIDController;
    private final ArmFeedforward mFFController;

    public AlgaeShooterPivot() {
        mKraken = new TalonFX(Settings.kAlgaePivotTalonID,"Canivore");
    
        var talonFXConfigurator = mKraken.getConfigurator();
        var motorConfigs = new MotorOutputConfigs();

        motorConfigs.Inverted = InvertedValue.Clockwise_Positive;
        talonFXConfigurator.apply(motorConfigs);

        mKraken.getConfigurator().apply(Settings.kAlgaePivotConfigs);

        mEncoder = new CANcoder(Settings.kAlgaePivotEncoder, "Canivore");

        mConstraints = new Constraints(
            Settings.kMaxAngularVelocity.getRadians(), 
            Settings.kMaxAngularAcceleration.getRadians()
        );
        mPPIDController = new ProfiledPIDController(
            Settings.kAlgaePivotConfigs.kP, 
            Settings.kAlgaePivotConfigs.kI, 
            Settings.kAlgaePivotConfigs.kD, 
            mConstraints
        );
        mFFController = new ArmFeedforward(
            Settings.kAlgaePivotConfigs.kS, 
            Settings.kAlgaePivotConfigs.kG, 
            Settings.kAlgaePivotConfigs.kV, 
            Settings.kAlgaePivotConfigs.kA
        );

        //mEncoder = mKraken.getAbsoluteEncoder();
        
        // Trapezoid profile with max velocity 80 rps, max accel 160 rps/s
        // final TrapezoidProfile m_profile = new TrapezoidProfile(
        // new TrapezoidProfile.Constraints(Settings.kMaxAngularVelocity, Settings.kMaxAngularAcceleration)
        // );

        // TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();

        // // create a position closed-loop request, voltage output, slot 0 configs
        // final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);

        // // calculate the next profile setpoint
        // m_setpoint = m_profile.calculate(0.020, m_setpoint, m_goal);

        // // send the request to the device
        // m_request.Position = m_setpoint.position;
        // m_request.Velocity = m_setpoint.velocity;
        // mKraken.setControl(m_request);
    }
    public static AlgaeShooterPivot getInstance() {
        if (mInstance == null) {
            mInstance = new AlgaeShooterPivot();
        }
        return mInstance;
    }

    public void setTargetAngle(Rotation2d angle) {
        mPPIDController.setGoal(angle.getRadians());
    }

    public Rotation2d getAngularVelocity() {
        // Default counts per revolution of the CANCoder
        double CPR = 4096.0;
        var rawVel = mEncoder.getVelocity().getValueAsDouble(); 
        var radps = (rawVel*20*Math.PI)/ CPR;
    
        return new Rotation2d(radps);
    }

    
    public Rotation2d getAngle() {
        var pos = mEncoder.getPosition().getValueAsDouble();

        return Rotation2d.fromDegrees(pos);
    }

    public void resetAngle() {
        mPPIDController.setGoal(getAngle().getRadians());
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter Pivot Angle (radians)", getAngle().getRadians());
        SmartDashboard.putNumber("Shooter Pivot Angular Velocity (radians / sec)", getAngularVelocity().getRadians());
        SmartDashboard.putNumber("Shooter Pivot Angle (degrees)", getAngle().getDegrees());
        SmartDashboard.putNumber("Shooter Pivot Angular Velocity (degrees / sec)", getAngularVelocity().getDegrees());
        SmartDashboard.putNumber("Profilled PID Controller Vel", mPPIDController.getSetpoint().velocity);

        double speed = mPPIDController.calculate(getAngle().getRadians());
        speed += mFFController.calculate(getAngle().getRadians() - Settings.kFFAngleOffset.getRadians(), mPPIDController.getSetpoint().velocity);

        SmartDashboard.putNumber("mPPIDC + mFFC Output", speed);

        mKraken.setVoltage(speed);
    }
}
