package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.Supplier;

public class CoralSubsystem extends SubsystemBase {
    public static class Settings {
        static final int kTalonPivotID = 11;
        static final int kCANcoderPivotID = 23;

        static final double kG = 0.19; // V
        static final double kS = 0.0; // V / rad
        static final double kV = 0; // V * sec / rad
        static final double kA = 0; // V * sec^2 / rad
//        static final double kV = 1.77; // V * sec / rad
//        static final double kA = 0.01; // V * sec^2 / rad

        static final Rotation2d kMaxVelocity = Rotation2d.fromDegrees(300);
        static final Rotation2d kMaxAcceleration = Rotation2d.fromDegrees(600);
        static final double kP = 4.0;
        static final double kI = 0.0;
        static final double kD = 0;

        static final double kZeroOffset = -0.126-.067; // rotations

        // TODO: Enable lower min-pos to bring down CoG when elevator is up. We should be able to tuck the shooter into the elevator.
//        static final Rotation2d kMinPos = Rotation2d.fromRotations(-0.025);
//        static final Rotation2d kMaxPos = Rotation2d.fromRotations(0.23);
    }

    public enum State {
        kHumanPlayer(Rotation2d.fromRotations(0.196)),
        kScoreV2(Rotation2d.fromRotations(-0.125732)),
        kScore(Rotation2d.fromRotations(0.06));

        State(Rotation2d pos) {
            this.pos = pos;
        }

        public final Rotation2d pos;
    }

    private final TalonFX mTalonPivot;
    private final CANcoder mCANcoderPivot;
    private final ArmFeedforward mFFController;
    private final ProfiledPIDController mPPIDController;

    private CoralSubsystem() {
        mTalonPivot = new TalonFX(Settings.kTalonPivotID);
        mTalonPivot.getConfigurator().apply(new TalonFXConfiguration().withMotorOutput(new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Coast)
        ));

        mCANcoderPivot = new CANcoder(Settings.kCANcoderPivotID);
        mCANcoderPivot.getConfigurator().apply(new CANcoderConfiguration().withMagnetSensor(new MagnetSensorConfigs().
                withSensorDirection(SensorDirectionValue.CounterClockwise_Positive).
                withMagnetOffset(Settings.kZeroOffset)
        ));

        mFFController = new ArmFeedforward(Settings.kS, Settings.kG, Settings.kV, Settings.kA);
        mPPIDController = new ProfiledPIDController(Settings.kP, Settings.kI, Settings.kD, new TrapezoidProfile.Constraints(
                Settings.kMaxVelocity.getRadians(),
                Settings.kMaxAcceleration.getRadians()
        ));

        setTargetState(State.kHumanPlayer);
    }


    private static CoralSubsystem mInstance;

    public static CoralSubsystem getInstance() {
        if (mInstance == null) {
            mInstance = new CoralSubsystem();
        }
        return mInstance;
    }

    public void setTargetState(State targetState) {
        setTargetPosition(targetState.pos);
    }

    public void setTargetPosition(Rotation2d targetPosition) {
        // NOTE: Use radians for target goal to align with re:calc constant units
        mPPIDController.setGoal(targetPosition.getRadians());
    }

    public Rotation2d getWristPosition() {
        var pos = mCANcoderPivot.getAbsolutePosition().getValueAsDouble();
        return Rotation2d.fromRotations(pos);
    }

    public Rotation2d getWristVelocity() {
        var vel = mCANcoderPivot.getVelocity().getValueAsDouble();
        return Rotation2d.fromRotations(vel);
    }

    @Override
    public void periodic() {
        var voltage = mPPIDController.calculate(getWristPosition().getRadians());
        voltage += mFFController.calculate(getWristPosition().getRadians(), mPPIDController.getSetpoint().velocity);
        mTalonPivot.setVoltage(voltage);

        // Telemetry
        SmartDashboard.putNumber("Coral Pivot Pos (rotations)", getWristPosition().getRotations());
        SmartDashboard.putNumber("Coral Pivot Target Pos (rotations)", Rotation2d.fromRadians(mPPIDController.getSetpoint().position).getRotations());
        SmartDashboard.putNumber("Coral Pivot Vel (rotations*sec^-1)", getWristVelocity().getRotations());
        SmartDashboard.putNumber("Coral Pivot Target Vel (rotations*sec^-1)", Rotation2d.fromRadians(mPPIDController.getSetpoint().velocity).getRotations());
        SmartDashboard.putNumber("Coral Pivot Applied Voltage", voltage);
    }

    public static class TuningCommand extends Command {
        Supplier<Double> ds;
        public TuningCommand(Supplier<Double> ds) {
            this.ds = ds;
            addRequirements(CoralSubsystem.getInstance());
        }

        @Override
        public void execute() {
            final var kMin = 0.0f;
            final var kMax = 0.2f;
            CoralSubsystem.getInstance().setTargetPosition(Rotation2d.fromRotations(ds.get() * 0.2));
        }
    }


    public static class DefaultCommand extends Command {
        public DefaultCommand() {
            addRequirements(CoralSubsystem.getInstance());
        }

        @Override
        public void execute() {
            CoralSubsystem.getInstance().setTargetState(State.kHumanPlayer);
        }
    }

    public static class SetStateCommand extends Command {
        private State state;
        public SetStateCommand(State state) {
            this.state = state;
            addRequirements(CoralSubsystem.getInstance());
        }

        @Override
        public void initialize() {
            CoralSubsystem.getInstance().setTargetState(state);
        }
    }
}
