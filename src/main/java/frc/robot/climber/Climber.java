package frc.robot.climber;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.rushinator.RushinatorPivot;

public class Climber extends SubsystemBase{
    public static class Settings {
        static final int kClimberPivotId = 27;

        static final InvertedValue kClimberPivotInverted = InvertedValue.Clockwise_Positive;

        static final double kG = 0.42; // V
        static final double kS = 0.0;  // V / rad
        static final double kV = 1.6; // V * sec / rad
        static final double kA = 0.0; // V * sec^2 / rad

        static final double kP = 7.0;
        static final double kI = 0.0;
        static final double kD = 0.0;

        public static final Rotation2d kMaxAngularVelocity = Rotation2d.fromDegrees(200); //120
        public static final Rotation2d kMaxAngularAcceleration = Rotation2d.fromDegrees(300);

        public static final Rotation2d kMaxPos = Rotation2d.fromRotations(0.8);
        public static final Rotation2d kMinPos = Rotation2d.fromRotations(0.0);

        public static final Rotation2d kAFFAngleOffset = Rotation2d.fromDegrees(0);

        static final double kCurrentLimit = 40.0;

    }

    public enum State {
        kDeploy(Rotation2d.fromRotations(-0.0185546875)),
        kRetract(Rotation2d.fromRotations(0.2568359375)),
        kStow(Settings.kMinPos);

        State(Rotation2d pos) {
            this.pos = pos;

        }
        public final Rotation2d pos;
    }

    public static Climber mInstance;

    private TalonFX ClimberPivot;
    private final ProfiledPIDController mPPIDController;
    private Constraints mConstraints;
    private final ArmFeedforward mAFFController;

    public static State kLastState;

    public Climber() {
        ClimberPivot = new TalonFX(Settings.kClimberPivotId);

        var ElbowPivotConfigurator = ClimberPivot.getConfigurator();

        var ElbowPivotConfigs = new MotorOutputConfigs();

        // set invert to CW+ and apply config change
        ElbowPivotConfigs.Inverted = Settings.kClimberPivotInverted;

        ElbowPivotConfigurator.apply(ElbowPivotConfigs);

        mPPIDController = new ProfiledPIDController(Settings.kP, Settings.kI, Settings.kD, mConstraints);
        mAFFController = new ArmFeedforward(Settings.kS, Settings.kG, Settings.kV, Settings.kA);
        mConstraints = new Constraints(Settings.kMaxAngularVelocity.getRadians(), Settings.kMaxAngularAcceleration.getRadians());

        ClimberPivot.getConfigurator().apply(new CurrentLimitsConfigs().withSupplyCurrentLimit(Settings.kCurrentLimit));
        
        if (kLastState == null) {
            kLastState = State.kStow;
        }
        mPPIDController.setGoal(kLastState.pos.getRotations());
    }

    public static Climber getInstance() {
        if (mInstance == null) {
            mInstance = new Climber();
        }
        return mInstance;
    }

    public void setTargetPos(Rotation2d pos) {
        mPPIDController.setGoal(pos.getRotations());
    }

    public Rotation2d getPos() {
        var pos = ClimberPivot.getPosition().getValueAsDouble();

        return Rotation2d.fromRotations(pos);
    }

    public Rotation2d getAngularVelocity() {
        return Rotation2d.fromRotations(ClimberPivot.getVelocity().getValueAsDouble());
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Climber Pivot Angle (Rotations)", getPos().getRotations());
        SmartDashboard.putNumber("Climber Pivot Angular Velocity (Rotations / sec)", getAngularVelocity().getRotations());

        SmartDashboard.putNumber("Climber Target Pos", mPPIDController.getSetpoint().position);
        SmartDashboard.putNumber("Target Vel", mPPIDController.getSetpoint().velocity);

        // Method to run pivots
        double speed = mPPIDController.calculate(getPos().getRotations());
        speed += mAFFController.calculate(getPos().getRotations(), mPPIDController.getSetpoint().velocity);

        SmartDashboard.putNumber("mPPIDC + mFFC Output", speed);

        // ClimberPivot.setVoltage(speed);
    }

    public static class DefaultCommand extends Command {

        public DefaultCommand() {
            addRequirements(RushinatorPivot.getInstance());
        }

        @Override
        public void execute() {
            Climber.getInstance().setTargetPos(State.kStow.pos);
        }

    }
}
