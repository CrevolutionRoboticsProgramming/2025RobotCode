package frc.robot.elevator;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;



public class Elevator extends SubsystemBase{
    public static class Settings {
        static final int kElevatorLeftId = 9;
        static final int kElevatorRightId = 10;

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

    public double getPosition() {
        var encoderRoataions = ElevatorLeft.getPosition().getValueAsDouble();
        double gearRatio = 4.0/8.0;
        double drumDiameter = 10.0;
        double drumCircum = Math.PI*drumDiameter;

        return (encoderRoataions*drumCircum) / gearRatio;
    }

    public double getVelocity() {
        double gearRatio = 4.0/8.0;
        double drumDiameter = 10.0;
        double drumCircum = Math.PI*drumDiameter;
        var rawVel = ElevatorLeft.getVelocity().getValueAsDouble(); 
    
        return rawVel * (drumCircum/gearRatio);
    }
    

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator Postion (meters)", getPosition());
        SmartDashboard.putNumber("Elevator Postion Velocity (meters / sec)", getVelocity());

        SmartDashboard.putNumber("Profilled PID Controller Vel", mPPIDController.getSetpoint().velocity);

        // Method to run pivots
        double speed = mPPIDController.calculate(getPosition());
        speed += mAFFController.calculate(getPosition() - Settings.kAFFAngleOffset.getRadians(), mPPIDController.getSetpoint().velocity);

        SmartDashboard.putNumber("mPPIDC + mFFC Output", speed);

        ElevatorLeft.setVoltage(speed);
        ElevatorRight.setVoltage(speed);
    }
}
