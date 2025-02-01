package frc.robot.WristPivot;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.coralator.Coralator;

public class WristPivot extends SubsystemBase{
    public static class Settings {
        static final int kWristPivotId = 28;

        static final InvertedValue kWristPivotInverted = InvertedValue.Clockwise_Positive;

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
        
    }
    public static WristPivot mInstance;

    private TalonFX ElevatorPivot, CoralPivot;
    private final ProfiledPIDController mPPIDController;
    private Constraints mConstraints;
    private final ArmFeedforward mAFFController;

    public WristPivot() {
        ElevatorPivot = new TalonFX(Settings.kWristPivotId);
        CoralPivot = new TalonFX(Settings.kWristPivotId);

        var ElevatorPivotConfigurator = ElevatorPivot.getConfigurator();
        var CoralPivotConfigurator = CoralPivot.getConfigurator();

        var ElevatorPivotConfigs = new MotorOutputConfigs();
        var CoralPivotConfigs = new MotorOutputConfigs();

        // set invert to CW+ and apply config change
        CoralPivotConfigs.Inverted = Settings.kWristPivotInverted;

        ElevatorPivotConfigurator.apply(ElevatorPivotConfigs);
        CoralPivotConfigurator.apply(CoralPivotConfigs);

        mPPIDController = new ProfiledPIDController(Settings.kP, Settings.kI, Settings.kD, mConstraints);
        mAFFController = new ArmFeedforward(Settings.kS, Settings.kG, Settings.kV, Settings.kA);
        mConstraints = new Constraints( Settings.kMaxAngularVelocity.getRadians(), Settings.kMaxAngularAcceleration.getRadians());
        
    }

    public static WristPivot getInstance() {
        if (mInstance == null) {
            mInstance = new WristPivot();
        }
        return mInstance;
    }
}
