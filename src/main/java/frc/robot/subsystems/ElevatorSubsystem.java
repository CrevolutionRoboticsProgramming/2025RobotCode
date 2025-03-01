package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.Supplier;

public class ElevatorSubsystem extends SubsystemBase {

    public static class Settings {
        static final int kTalonLeftID = 9;
        static final int kTalonRightID = 10;

//        static final InvertedValue kElevatorInverted = InvertedValue.Clockwise_Positive;

//        static final int kCanCoderId = 6;

//        static final double kG = 0.42; // V
//        static final double kS = 0.0;  // V / rad
//        static final double kV = 1.6; // V * sec / rad
//        static final double kA = 0.0; // V * sec^2 / rad
//
//        static final double kP = 7.0;
//        static final double kI = 0.0;
//        static final double kD = 0.0;

//        public static final Rotation2d kMaxAngularVelocity = Rotation2d.fromDegrees(200); //120
//        public static final Rotation2d kMaxAngularAcceleration = Rotation2d.fromDegrees(300);
//        public static final Rotation2d kMaxAngle = Rotation2d.fromDegrees(180);
//        public static final Rotation2d kMaxAnglePhysical = Rotation2d.fromDegrees(175);
//
//        public static final Rotation2d kAFFAngleOffset = Rotation2d.fromDegrees(0);
    }

    private static ElevatorSubsystem mInstance;

    private TalonFX mTalonLeft, mTalonRight;

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
    }

    public void setVoltage(double voltage) {
        mTalonLeft.setVoltage(voltage);
        mTalonRight.setVoltage(voltage);
    }

    public double getPosition() {
        return mTalonRight.getPosition().getValueAsDouble();
    }

    public static ElevatorSubsystem getInstance() {
        if (mInstance == null) {
            mInstance = new ElevatorSubsystem();
        }
        return mInstance;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator Position", getPosition());
    }

    public static class DefaultCommand extends Command {
        private ElevatorSubsystem subsystem;
        private Supplier<Double> percentSupplier;

        private final double kBound = 3.0f;
        private final double kMaxPosition = 40.0f;

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
}
