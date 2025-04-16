package frc.robot.climber;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.rushinator.RushinatorPivot;

public class Climber extends SubsystemBase{
    public static class Settings {
        static final int kClimberPivotId = 27; //TODO: change this to sparkmax ID

        // static final InvertedValue kClimberPivotInverted = InvertedValue.Clockwise_Positive;
        static final boolean kClimberPivotInverted = false;

        static final int kStallCurrentLimit = 80; // in amps

        static final double kG = 0.1; // V
        static final double kS = 0.0;  // V / rad
        static final double kV = 0.0; // V * sec / rad
        static final double kA = 0.0; // V * sec^2 / rad

        static final double kP = 0.01;
        static final double kI = 0.0;
        static final double kD = 0.0;

        public static final Rotation2d kMaxVelocity = Rotation2d.fromDegrees(200); //120
        public static final Rotation2d kMaxAcceleration = Rotation2d.fromDegrees(300);

        public static final Rotation2d kMaxPos = Rotation2d.fromRotations(0.8); //TODO: change this
        public static final Rotation2d kMinPos = Rotation2d.fromRotations(0.0); //TODO: change this

        public static final Rotation2d kAFFAngleOffset = Rotation2d.fromDegrees(0);

        static final double kCurrentLimit = 40.0;

    }

    public enum State {
        //TODO: redo these states based on the climber encoder position
        kDeploy(Rotation2d.fromRotations(150.91650390625)), //TODO
        kRetract(Rotation2d.fromRotations(80.5693359375)), //TODO
        kStow(Settings.kMinPos); //TODO

        State(Rotation2d pos) {
            this.pos = pos;

        }
        public final Rotation2d pos;
    }

    public static Climber mInstance;

    // private TalonFX ClimberPivot;
    // private final ProfiledPIDController mPPIDController;
    private Constraints mConstraints;
    private final BangBangController mBBController;
    private final ArmFeedforward mAFFController;

    private SparkMax mClimberPivotMotor;
    private SparkMaxConfig mClimberPivotMotorConfig;
    private RelativeEncoder mClimberPivotMotorEncoder;


    public static State kLastState;

    public Climber() {
        mClimberPivotMotor = new SparkMax(Settings.kClimberPivotId, MotorType.kBrushless);

        mClimberPivotMotorConfig = new SparkMaxConfig();
        mClimberPivotMotorConfig.inverted(Settings.kClimberPivotInverted);
        mClimberPivotMotorConfig.smartCurrentLimit(Settings.kStallCurrentLimit);
        mClimberPivotMotorConfig.idleMode(IdleMode.kBrake);
        mClimberPivotMotor.configure(mClimberPivotMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        mClimberPivotMotorEncoder = mClimberPivotMotor.getEncoder();

        mAFFController = new ArmFeedforward(Settings.kS, Settings.kG, Settings.kV, Settings.kA);

        mBBController = new BangBangController(0.01);

        if (kLastState == null) {
            kLastState = State.kStow;
        }
        // mPPIDController.setGoal(kLastState.pos.getRotations());
        

        // ClimberPivot = new TalonFX(Settings.kClimberPivotId);

        // var ElbowPivotConfigurator = ClimberPivot.getConfigurator();

        // var ElbowPivotConfigs = new MotorOutputConfigs();

        // ClimberPivot.getConfigurator().apply(new TalonFXConfiguration().withMotorOutput(new MotorOutputConfigs()
        //         .withInverted(InvertedValue.Clockwise_Positive)
        //         .withNeutralMode(NeutralModeValue.Brake)
        // ));
        // ClimberPivot.getConfigurator().apply(new CurrentLimitsConfigs().withSupplyCurrentLimit(Settings.kCurrentLimit));

        // set invert to CW+ and apply config change
        // ElbowPivotConfigs.Inverted = Settings.kClimberPivotInverted;

        // ElbowPivotConfigurator.apply(ElbowPivotConfigs);

        // mPPIDController = new ProfiledPIDController(Settings.kP, Settings.kI, Settings.kD, new TrapezoidProfile.Constraints(
        //         Settings.kMaxVelocity.getRadians(),
        //         Settings.kMaxAcceleration.getRadians()
        // ));
        // mPPIDController.setTolerance(0.01);
        

        // ClimberPivot.setPosition(0.0);

        // ClimberPivot.getConfigurator().apply(new CurrentLimitsConfigs().withSupplyCurrentLimit(Settings.kCurrentLimit));
    }

    public static Climber getInstance() {
        if (mInstance == null) {
            mInstance = new Climber();
        }
        return mInstance;
    }

    public void setTargetPos(Rotation2d pos) {
        // mPPIDController.setGoal(pos.getRotations());
        mBBController.setSetpoint(pos.getRotations());
    }

    public Rotation2d getPos() {
        return Rotation2d.fromRotations(mClimberPivotMotorEncoder.getPosition());
    }

    public Rotation2d getAngularVelocity() {
        // return Rotation2d.fromRotations(ClimberPivot.getVelocity().getValueAsDouble());
        return Rotation2d.fromRotations(mClimberPivotMotorEncoder.getVelocity()); //TODO: i dont know what the unit conversions are
    }

    public static class DefaultCommand extends Command {

        public DefaultCommand() {
            addRequirements(Climber.getInstance());
        }

        @Override
        public void execute() {
            Climber.getInstance().setTargetPos(State.kStow.pos);
        }

    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Climber Pivot Angle (Rotations)", getPos().getRotations());
        SmartDashboard.putNumber("Climber Pivot Angular Velocity (Rotations / sec)", getAngularVelocity().getRotations() * 60.0);

        // SmartDashboard.putNumber("Climber Target Pos", mPPIDController.getSetpoint().position);
        // SmartDashboard.putNumber("Target Vel", mPPIDController.getSetpoint().velocity);

        SmartDashboard.putNumber("Climber Target Pos", mBBController.getSetpoint());
        SmartDashboard.putNumber("Target Vel (RPS)", mBBController.getSetpoint() / 60.0);

        // Method to run pivots
        double speed = mBBController.calculate(getPos().getRotations());
        speed += mAFFController.calculate(getPos().getRotations(), mBBController.getSetpoint() / 60.0);

        SmartDashboard.putNumber("mPPIDC + mFFC Output", speed);
        
        mClimberPivotMotor.setVoltage(speed);

        // ClimberPivot.setVoltage(speed);
    }
}
