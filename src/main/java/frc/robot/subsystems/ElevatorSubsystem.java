package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.elevator.Elevator;

import java.util.function.Supplier;

public class ElevatorSubsystem extends SubsystemBase {

    public static class Settings {
        static final int kTalonLeftID = 9;
        static final int kTalonRightID = 10;

        static final InvertedValue kElevatorInverted = InvertedValue.Clockwise_Positive;

        static final int kCanCoderId = 6;

        static final double kG = 0.01; // V
        static final double kS = 0.0;  // V / rad
        static final double kV = 6.3; // V * sec / rad
        static final double kA = 0.0; // V * sec^2 / rad

        static final double kP = 0.0;
        static final double kI = 0.0;
        static final double kD = 0.0;

        static final Rotation2d kMaxVelocity = Rotation2d.fromDegrees(300);
        static final Rotation2d kMaxAcceleration = Rotation2d.fromDegrees(600);
    }

    private static ElevatorSubsystem mInstance;

    private TalonFX mTalonLeft, mTalonRight;
    private final ElevatorFeedforward mFFController;
    private final ProfiledPIDController mPPIDController;

    public enum State {
        kCoralL1(Rotation2d.fromRotations(19.934082)),
        kCoralL2(Rotation2d.fromRotations(25.77832)),
        kCoralL3(Rotation2d.fromRotations(37.401367)),
        kAlgaeL2(Rotation2d.fromRotations(23.348633)),
        kAlgaeL3(Rotation2d.fromRotations(35.089355)),
        kZero(Rotation2d.fromRotations(0.0));

        State(Rotation2d pos) {
            this.pos = pos;
        }

        public final Rotation2d pos;
    }

    private ElevatorSubsystem() {
        mTalonLeft = new TalonFX(Settings.kTalonLeftID);
        mTalonRight = new TalonFX(Settings.kTalonRightID);

        mTalonLeft.getConfigurator().apply(new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake)
        );
        mTalonRight.getConfigurator().apply(new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake)
        );
        mTalonRight.setPosition(0);

        mFFController = new ElevatorFeedforward(Settings.kS, Settings.kG, Settings.kV, Settings.kA);
        mPPIDController = new ProfiledPIDController(Settings.kP, Settings.kI, Settings.kD, new TrapezoidProfile.Constraints(
                Settings.kMaxVelocity.getRadians(),
                Settings.kMaxAcceleration.getRadians()
        ));

    }

    public void setVoltage(double voltage) {
        mTalonLeft.setVoltage(voltage);
        mTalonRight.setVoltage(voltage);
    }

    public void setTargetState(State targetState) {
        setTargetPosition(targetState.pos);
    }

    public void setTargetPosition(Rotation2d targetPosition) {
        // NOTE: Use radians for target goal to align with re:calc constant units
        mPPIDController.setGoal(targetPosition.getRadians());
    }

    public double getPosition() {
        return mTalonRight.getPosition().getValueAsDouble();
    }

    public double getVelocity() {
        return mTalonRight.getVelocity().getValueAsDouble();
    }

    // public void setVoltageSupplier(Supplier<Double> voltageSupplier)

    public static ElevatorSubsystem getInstance() {
        if (mInstance == null) {
            mInstance = new ElevatorSubsystem();
        }
        return mInstance;
    }

    @Override
    public void periodic() {
        // var voltage = mPPIDController.calculate(getPosition());
        // voltage += mFFController.calculate(getPosition(), mPPIDController.getSetpoint().velocity);
        // setVoltage(voltage);

        // Telemetry
        SmartDashboard.putNumber("Elevator Pos (rotations)", getPosition());
        // SmartDashboard.putNumber("Elevator Target Pos (rotations)", Rotation2d.fromRadians(mPPIDController.getSetpoint().position).getRotations());
        SmartDashboard.putNumber("Elevator Vel (rotations*sec^-1)", getVelocity());
        // SmartDashboard.putNumber("Elevator Target Vel (rotations*sec^-1)", Rotation2d.fromRadians(mPPIDController.getSetpoint().velocity).getRotations());
        // SmartDashboard.putNumber("Elevator Applied Voltage", voltage);
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

    public static class VelocityCommand extends Command {
        private ElevatorSubsystem subsystem;
        private Supplier<Double> percentSupplier;

        private final double kBound = 3.0f;
        private final double kMaxPosition = 40.0f;

        private final double kMaxVelocity = 15.0f; // Rotations / Second
        ElevatorFeedforward mFFController;

        public VelocityCommand(ElevatorSubsystem subsystem, Supplier<Double> percentSupplier) {
            this.subsystem = subsystem;
            this.percentSupplier = percentSupplier;
            addRequirements(subsystem);

            mFFController = new ElevatorFeedforward(0.0, 0.0, 0.2);
        }

        @Override
        public void execute() {
            var targetVelocity = percentSupplier.get() * kMaxVelocity;
            if (subsystem.getPosition() <= 0 && targetVelocity < 0) {
                targetVelocity = 0.0;
            } 
            if (subsystem.getPosition() >= kMaxPosition && targetVelocity > 0) {
                targetVelocity = 0.0;
            }

            final var voltage = mFFController.calculate(targetVelocity);
            subsystem.setVoltage(voltage);

            SmartDashboard.putNumber("Elevator Target Velocity (rotations)", targetVelocity);
            SmartDashboard.putNumber("Elevato Applied Voltage", voltage);
        }

        @Override
        public void end(boolean interrupted) {
            subsystem.setVoltage(0);
        }
    }
}
