package frc.robot.ElbowPivot;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.coralator.Coralator;

public class ElbowPivot extends SubsystemBase{
    public static class Settings {
        static final int kElbowPivotId = 27;

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
        
    }
    public static ElbowPivot mInstance;

    private TalonFX ElevatorPivot, CoralPivot;
    private final ProfiledPIDController mPPIDController;
    private Constraints mConstraints;
    private final ArmFeedforward mAFFController;

    public ElbowPivot() {
        ElevatorPivot = new TalonFX(Settings.kElbowPivotId);

        var ElevatorPivotConfigurator = ElevatorPivot.getConfigurator();
        var CoralPivotConfigurator = CoralPivot.getConfigurator();

        var ElevatorPivotConfigs = new MotorOutputConfigs();
        var CoralPivotConfigs = new MotorOutputConfigs();

        // set invert to CW+ and apply config change
        ElevatorPivotConfigs.Inverted = Settings.kElevatorPivotInverted;

        ElevatorPivotConfigurator.apply(ElevatorPivotConfigs);
        CoralPivotConfigurator.apply(CoralPivotConfigs);

        mPPIDController = new ProfiledPIDController(Settings.kP, Settings.kI, Settings.kD, mConstraints);
        mAFFController = new ArmFeedforward(Settings.kS, Settings.kG, Settings.kV, Settings.kA);
        mConstraints = new Constraints( Settings.kMaxAngularVelocity.getRadians(), Settings.kMaxAngularAcceleration.getRadians());
        
    }

    public static ElbowPivot getInstance() {
        if (mInstance == null) {
            mInstance = new ElbowPivot();
        }
        return mInstance;
    }
}
