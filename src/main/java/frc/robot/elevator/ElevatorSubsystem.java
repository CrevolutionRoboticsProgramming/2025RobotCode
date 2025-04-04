package frc.robot.elevator;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.Supplier;

import javax.sound.midi.MetaEventListener;

public class ElevatorSubsystem extends SubsystemBase {

    public static class Settings {
        static final int kTalonLeftID = 9;
        static final int kTalonRightID = 10;

        static final int kLowerLimitSwitch = 0;

        static final InvertedValue kElevatorInverted = InvertedValue.Clockwise_Positive;

        static final int kCanCoderId = 6;

        static final double kGLow = 0.2; // V
        static final double kGHigh = 0.25; // V
        static final double kS = 0.0;  // V / rad
        static final double kV = 0.125; // V * sec / rad
        static final double kA = 0.0; // V * sec^2 / rad

        static final double kP = 0.15;
        static final double kI = 0.0;
        static final double kD = 0.0;

//        static final Rotation2d kMaxVelocity = Rotation2d.fromDegrees(300);
//        static final Rotation2d kMaxAcceleration = Rotation2d.fromDegrees(600);

        static final double kCurrentLimit = 40.0;

        static final double kMaxVelocity = 250.0;
        static final double kMaxAcceleration = 250.0;

        static final double kCrossoverPoint = 17.2939453125;
    }

    private static ElevatorSubsystem mInstance;

    private TalonFX mTalonLeft, mTalonRight;
    private final ElevatorFeedforward mFFLowController, mFFHighController;
    public final ProfiledPIDController mPPIDController;
    private DigitalInput mLowerLimitSwitch;

    private Supplier<Double> mVelocitySupplier;


    public enum State {
        kCoralL1(0.0),
        kCoralL2(0.0),
        kCoralScoreL2(6.3461914062 - 1.0),
        kCoralL3(12.54296875),
        kCoralScoreL3(15.7583007812),
        kCoralL4AutonScore(16.2),
        kCoralL4(38.05810546875),
        kCoralScoreL4(35.4150390625),
        kAlgaeL2(17.01552734375),
        kAlgaeL3(30.27490234375),
        kAlgaeIntake(3.650390625),
        kClimb(13.1376953125),
        kZero(0.0);

        State(double pos) {
            this.pos = pos;
        }

        public final double pos;
    }

    public static State kLastState;

    private ElevatorSubsystem() {
        mTalonLeft = new TalonFX(Settings.kTalonLeftID);
        mTalonRight = new TalonFX(Settings.kTalonRightID);

        mTalonLeft.getConfigurator().apply(new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake)
        );
        mTalonLeft.getConfigurator().apply(new CurrentLimitsConfigs().withSupplyCurrentLimit(Settings.kCurrentLimit));

        mTalonRight.getConfigurator().apply(new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake)
        );
        mTalonRight.getConfigurator().apply(new CurrentLimitsConfigs().withSupplyCurrentLimit(Settings.kCurrentLimit));
        mTalonRight.setPosition(0);

        // mLowerLimitSwitch = new DigitalInput(Settings.kLowerLimitSwitch);

        mFFLowController = new ElevatorFeedforward(Settings.kS, Settings.kGLow, Settings.kV, Settings.kA);
        mFFHighController = new ElevatorFeedforward(Settings.kS, Settings.kGHigh, Settings.kV, Settings.kA);
        mPPIDController = new ProfiledPIDController(Settings.kP, Settings.kI, Settings.kD, new TrapezoidProfile.Constraints(
                Settings.kMaxVelocity,
                Settings.kMaxAcceleration
        ));

        if(kLastState == null) {
            kLastState = State.kZero;
        }

        mVelocitySupplier = null;
    }

    public void setVoltage(double voltage) {
        mTalonLeft.setVoltage(voltage);
        mTalonRight.setVoltage(voltage);
    }

    public void setTargetState(State targetState) {
        kLastState = targetState;
        setTargetPosition(targetState.pos);
    }

    public void setTargetPosition(double pos) {
        // NOTE: Use radians for target goal to align with re:calc constant units
        mPPIDController.setGoal(pos);
    }

    public double getPosition() {
        return mTalonRight.getPosition().getValueAsDouble();
    }

    public double getVelocity() {
        return mTalonRight.getVelocity().getValueAsDouble();
    }

    public void setVelocitySupplier(Supplier<Double> velocitySupplier) {
        mVelocitySupplier = velocitySupplier;
    }

    // public boolean getLowerLimitSwitch() {
    //     return mLowerLimitSwitch.get();
    // }

    // public void resetRelEncoder() {
    //     mTalonRight.setPosition(0.0);
    // }

    public static ElevatorSubsystem getInstance() {
        if (mInstance == null) {
            mInstance = new ElevatorSubsystem();
        }
        return mInstance;
    }

    private double getFeedforwardOutput(double targetVelocity) {
        if (getPosition() < Settings.kCrossoverPoint) {
            return mFFLowController.calculate(targetVelocity);
        }
        return mFFHighController.calculate(targetVelocity);
    }

    @Override
    public void periodic() {
        // if (mVelocitySupplier == null) {
        //     voltage = mPPIDController.calculate(getPosition());
        //     targetVelocity = mPPIDController.getSetpoint().velocity;
        //     voltage += getFeedforwardOutput(targetVelocity);
        // } else {
        //     targetVelocity = mVelocitySupplier.get();
        //     voltage = getFeedforwardOutput(targetVelocity);
        // }

        // if (getLowerLimitSwitch()) {
        //     resetRelEncoder();
        // }

        double voltage = 0.0;
        voltage = mPPIDController.calculate(getPosition());
        voltage += getFeedforwardOutput(mPPIDController.getSetpoint().velocity);
        setVoltage(voltage);

        // Telemetry
        SmartDashboard.putNumber("Elevator Position", getPosition());
        SmartDashboard.putNumber("Elevator Velocity", getVelocity());
        if (mVelocitySupplier != null) {
            SmartDashboard.putNumber("Elevator Target Position", mPPIDController.getSetpoint().position);
        }

        SmartDashboard.putString("KLastState Elevator", kLastState.name());
        SmartDashboard.putNumber("Elevator Target Velocity", mPPIDController.getSetpoint().velocity);
        SmartDashboard.putNumber("Elevator Applied Voltage", voltage);
        SmartDashboard.putString("Elevator Mode", (mVelocitySupplier == null) ? "PPID" : "Manual");
    }

    public static class DefaultCommand extends Command {
        private ElevatorSubsystem subsystem;
        private Supplier<Double> percentSupplier;

        private final double kBound = 3.0f;
        private final double kMaxPosition = 40.0f;

        private final double kMaxVelocity = 10.0f;

        public DefaultCommand(ElevatorSubsystem subsystem, Supplier<Double> percentSupplier) {
            this.subsystem = subsystem;
            this.percentSupplier = percentSupplier;
            addRequirements(subsystem);
        }

        @Override
        public void execute() {
            var targetVoltage = percentSupplier.get();
            if (targetVoltage > 0) {
                targetVoltage *= 3;
            } else {
                targetVoltage *= 2;
            }

            var pos = subsystem.getPosition();
            if (pos < 0) {
                pos = 0;
            } else if (pos > kMaxPosition) {
                pos = kMaxPosition;
            }

            if (pos < kBound && targetVoltage < 0) {
                targetVoltage *= (pos / kBound);
            } else if (pos > (kMaxPosition - kBound) && targetVoltage > 0) {
                targetVoltage *= (kMaxPosition - pos) / kBound;
            }
            subsystem.setVoltage(targetVoltage);
        }
    }

    public static class applyJog extends Command {
        private ElevatorSubsystem mElevatorSubsystem;
        private double target;
        public applyJog(double targetPos) {
            mElevatorSubsystem = ElevatorSubsystem.getInstance();
            target = targetPos;
            addRequirements(mElevatorSubsystem);
        }

        @Override
        public void initialize() {
            mElevatorSubsystem.setTargetPosition(target);
        }
    }

    public static class zero extends Command {
        private ElevatorSubsystem mElevatorSubsystem;
        public zero() {
            mElevatorSubsystem = ElevatorSubsystem.getInstance();
            addRequirements(mElevatorSubsystem);
        }

        @Override
        public void initialize() {
            mElevatorSubsystem.setTargetPosition(ElevatorSubsystem.State.kZero.pos);
        }
    }

    // public static class VelocityCommand extends Command {
    //     private ElevatorSubsystem subsystem;
    //     private Supplier<Double> percentSupplier;

    //     private final double kMaxPosition = 40.0f;

    //     public VelocityCommand(ElevatorSubsystem subsystem, Supplier<Double> percentSupplier) {
    //         this.subsystem = subsystem;
    //         this.percentSupplier = percentSupplier;
    //         addRequirements(subsystem);
    //     }

    //     private double getTargetVelocity() {
    //         var targetVelocity = percentSupplier.get() * Settings.kMaxVelocity;
    //         if (subsystem.getPosition() <= 0 && targetVelocity < 0) {
    //             targetVelocity = 0.0;
    //         }
    //         if (subsystem.getPosition() >= kMaxPosition && targetVelocity > 0) {
    //             targetVelocity = 0.0;
    //         }
    //         return targetVelocity;
    //     }

    //     @Override
    //     public void initialize() {
    //         subsystem.setVelocitySupplier(this::getTargetVelocity);
    //     }

    //     @Override
    //     public void end(boolean interrupted) {
    //         // By setting the velocity supplier to null, we re-enable the default PID controller.
    //         // We should also update the setpoint to the current position to avoid sending the elevator shooting to the
    //         // previous setpoint.
    //         subsystem.setTargetPosition(subsystem.getPosition());
    //         subsystem.setVelocitySupplier(null);
    //     }
    // }
}
