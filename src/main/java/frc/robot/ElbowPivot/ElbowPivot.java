package frc.robot.ElbowPivot;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.coralator.Coralator;

public class ElbowPivot extends SubsystemBase{
    public static class Settings {
        static final int kElbowPivotId = 27;

        static final int kCanCoderId = 2;

        static final InvertedValue kElevatorPivotInverted = InvertedValue.Clockwise_Positive;

        static final double kG = 0.42; // V
        static final double kS = 0.0;  // V / rad
        static final double kV = 1.6; // V * sec / rad
        static final double kA = 0.0; // V * sec^2 / rad

        static final double kP = 7.0;
        static final double kI = 0.0;
        static final double kD = 0.0;

        public static final Rotation2d kMaxAngularVelocity = Rotation2d.fromDegrees(200); //120
        public static final Rotation2d kMaxAngularAcceleration = Rotation2d.fromDegrees(300);
        public static final Rotation2d kMaxAngle = Rotation2d.fromDegrees(180);
        public static final Rotation2d kMaxAnglePhysical = Rotation2d.fromDegrees(175);

        public static final Rotation2d kAFFAngleOffset = Rotation2d.fromDegrees(0);

    }
    public static ElbowPivot mInstance;

    private TalonFX ElbowPivot, CoralPivot;
    private final ProfiledPIDController mPPIDController;
    private Constraints mConstraints;
    private final ArmFeedforward mAFFController;
    private final CANcoder mCanCoder;

    private Rotation2d lastRequestedVelocity;

    public ElbowPivot() {
        ElbowPivot = new TalonFX(Settings.kElbowPivotId, "Canivore");
        mCanCoder = new CANcoder(Settings.kCanCoderId, "Canivore");

        var ElbowPivotConfigurator = ElbowPivot.getConfigurator();

        var ElbowPivotConfigs = new MotorOutputConfigs();

        // set invert to CW+ and apply config change
        ElbowPivotConfigs.Inverted = Settings.kElevatorPivotInverted;

        ElbowPivotConfigurator.apply(ElbowPivotConfigs);

        mPPIDController = new ProfiledPIDController(Settings.kP, Settings.kI, Settings.kD, mConstraints);
        mAFFController = new ArmFeedforward(Settings.kS, Settings.kG, Settings.kV, Settings.kA);
        mConstraints = new Constraints(Settings.kMaxAngularVelocity.getRadians(), Settings.kMaxAngularAcceleration.getRadians());
        
    }

    public static ElbowPivot getInstance() {
        if (mInstance == null) {
            mInstance = new ElbowPivot();
        }
        return mInstance;
    }

    public void setTargetAngle(Rotation2d angle) {
        mPPIDController.setGoal(angle.getRadians());
    }

    public Rotation2d getAngle() {
        var pos = mCanCoder.getPosition().getValueAsDouble();

        return Rotation2d.fromDegrees(pos);
    }

    public Rotation2d getAngularVelocity() {
        // Default counts per revolution of the CANCoder
        double CPR = 4096.0;
        var rawVel = mCanCoder.getVelocity().getValueAsDouble(); 
        var radps = (rawVel*20*Math.PI)/ CPR;
    
        return new Rotation2d(radps);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter Pivot Angle (radians)", getAngle().getRadians());
        SmartDashboard.putNumber("Shooter Pivot Angular Velocity (radians / sec)", getAngularVelocity().getRadians());

        SmartDashboard.putNumber("Shooter Pivot Angle (degrees)", getAngle().getDegrees());
        SmartDashboard.putNumber("Shooter Pivot Angular Velocity (degrees / sec)", getAngularVelocity().getDegrees());

        SmartDashboard.putNumber("Profilled PID Controller Vel", mPPIDController.getSetpoint().velocity);

        // Method to run pivots
        double speed = mPPIDController.calculate(getAngle().getRadians());
        speed += mAFFController.calculate(getAngle().getRadians() - Settings.kAFFAngleOffset.getRadians(), mPPIDController.getSetpoint().velocity);

        SmartDashboard.putNumber("mPPIDC + mFFC Output", speed);

        ElbowPivot.setVoltage(speed);
    }
}
