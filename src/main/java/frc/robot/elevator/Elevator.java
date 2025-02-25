package frc.robot.elevator;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.WristPivot.WristPivot;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkMaxAlternateEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;



public class Elevator extends SubsystemBase{
    public static class Settings {
        static final int kElevatorLeftId = 28;
        static final int kElevatorRightId = 28;

        static final InvertedValue kElevatorInverted = InvertedValue.Clockwise_Positive;

        static final int kCanCoderId = 6;

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
    public static Elevator mInstance;
    
    private TalonFX ElevatorLeft, ElevatorRight;
    private final ProfiledPIDController mPPIDController;
    private Constraints mConstraints;
    private final ArmFeedforward mAFFController;

    // public ElevatorState desiredState;

    public Elevator() {
        ElevatorLeft = new TalonFX(Settings.kElevatorLeftId);
        ElevatorRight = new TalonFX(Settings.kElevatorRightId);

        var ElevatorLeftConfigurator = ElevatorLeft.getConfigurator();
        var ElevatorRightConfigurator = ElevatorRight.getConfigurator();

        var ElevatorLeftConfigs = new MotorOutputConfigs();
        var ElevatorRightConfigs = new MotorOutputConfigs();

        // set invert to CW+ and apply config change
        ElevatorLeftConfigs.Inverted = Settings.kElevatorInverted;
        ElevatorRightConfigs.Inverted = Settings.kElevatorInverted;

        ElevatorLeftConfigurator.apply(ElevatorLeftConfigs);
        ElevatorRightConfigurator.apply(ElevatorRightConfigs);

        mPPIDController = new ProfiledPIDController(Settings.kP, Settings.kI, Settings.kD, mConstraints);
        mAFFController = new ArmFeedforward(Settings.kS, Settings.kG, Settings.kV, Settings.kA);
        mConstraints = new Constraints( Settings.kMaxAngularVelocity.getRadians(), Settings.kMaxAngularAcceleration.getRadians());
    }

    public static Elevator getInstance() {
        if (mInstance == null) {
            mInstance = new Elevator();
        }
        return mInstance;
    }

    public void setTargetPosition(Rotation2d angle) {
        mPPIDController.setGoal(angle.getRadians());
    }

    public Rotation2d getPosition() {
        var pos = ElevatorLeft.getPosition().getValueAsDouble();

        return Rotation2d.fromDegrees(pos);
    }

    public Rotation2d getAngularVelocity() {
        // Default counts per revolution of the CANCoder
        double CPR = 4096.0;
        var rawVel = ElevatorLeft.getVelocity().getValueAsDouble(); 
        var radps = (rawVel*20*Math.PI)/ CPR;
    
        return new Rotation2d(radps);
    }
    

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator Postion (meters)", getPosition().getRadians());
        SmartDashboard.putNumber("Elevator Postion Velocity (meters / sec)", getAngularVelocity().getRadians());

        SmartDashboard.putNumber("Profilled PID Controller Vel", mPPIDController.getSetpoint().velocity);

        // Method to run pivots
        double speed = mPPIDController.calculate(getPosition().getRadians());
        speed += mAFFController.calculate(getPosition().getRadians() - Settings.kAFFAngleOffset.getRadians(), mPPIDController.getSetpoint().velocity);

        SmartDashboard.putNumber("mPPIDC + mFFC Output", speed);

        ElevatorLeft.setVoltage(speed);
        ElevatorRight.setVoltage(speed);
    }
}
