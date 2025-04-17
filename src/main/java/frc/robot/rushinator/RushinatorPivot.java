package frc.robot.rushinator;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
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
import frc.robot.algaepivot.AlgaeSubsystem;

public class RushinatorPivot extends SubsystemBase {
    public static class Settings {
        static final int kTalonPivotID = 11;
        static final int kCANcoderPivotID = 23;

        static final double kG = 0.1; // V
        static final double kS = 0.0; // V / rad
        static final double kV = 5.0; // V * sec / rad
        static final double kA = 1.77; // V * sec^2 / rad

        static final Rotation2d kMaxVelocity = Rotation2d.fromDegrees(10000);
        static final Rotation2d kMaxAcceleration = Rotation2d.fromDegrees(10000);
        static final double kP = 38.0;          
        static final double kI = 0.0;
        static final double kD = 0.0;

        static final double kZeroOffset = 0.037841796875; // rotations

        static final double kCurrentLimit = 40.0;

        // TODO: Enable lower min-pos to bring down CoG when elevator is up. We should be able to tuck the shooter into the elevator.
        static final Rotation2d kMinPos = Rotation2d.fromRotations(-0.02128);
        static final Rotation2d kMaxPos = Rotation2d.fromRotations(0.3218);
    }

    public enum State {
        kFloorIntake(Rotation2d.fromRotations(-0.08349609375)),
        kHPIntake(Rotation2d.fromRotations(0.211669921875)),
        kScore(Rotation2d.fromRotations(0.119873046875)),
        kScoreL1(Rotation2d.fromRotations(0.10595703125)),
        kScoreL2(Rotation2d.fromRotations(0.04272460937499999)),
        kScoreL3(Rotation2d.fromRotations(0.079833984375)),
        kScoreL4(Rotation2d.fromRotations(0.06058984375)),
        kStowL4(Rotation2d.fromRotations(0.146728515625)),
        kStowAutoAlignL4(Rotation2d.fromRotations(0.185)),
        kScoreL4Auton(Rotation2d.fromRotations(0.10670898437)),
        kStowTravel(Rotation2d.fromRotations(0.223876953125)),
        kLoliPop(Rotation2d.fromRotations(-0.075419921875)),
        kClimb(Rotation2d.fromRotations(0.27685546875)),
        kTuck(Settings.kMaxPos);

        State(Rotation2d pos) {
            this.pos = pos;

        }
        public final Rotation2d pos;
    }

    private final TalonFX mTalonPivot;
    private final CANcoder mCANcoderPivot;
    private final ArmFeedforward mFFController;
    public final ProfiledPIDController mPPIDController;

    public static State kLastState;

    private RushinatorPivot() {
        mTalonPivot = new TalonFX(Settings.kTalonPivotID);
        mTalonPivot.getConfigurator().apply(new TalonFXConfiguration().withMotorOutput(new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake)
        ));
        mTalonPivot.getConfigurator().apply(new CurrentLimitsConfigs().withSupplyCurrentLimit(Settings.kCurrentLimit));

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
        mPPIDController.setTolerance(0.01);

        if (kLastState == null) {
            kLastState = State.kStowTravel;
        }
        mPPIDController.setGoal(kLastState.pos.getRotations());
    }


    private static RushinatorPivot mInstance;
    public static RushinatorPivot getInstance() {
        if (mInstance == null) {
            mInstance = new RushinatorPivot();
        }
        return mInstance;
    }

    public void setTargetState(State targetState) {
        kLastState = targetState;
        setTargetPosition(targetState.pos);
    }

    public Rotation2d getPivotAngle() {
        return Rotation2d.fromRotations(mCANcoderPivot.getAbsolutePosition().getValueAsDouble() * 96);
    }


    public void setTargetPosition(Rotation2d targetPosition) {
        // NOTE: Use radians for target goal to align with re:calc constant units
        mPPIDController.setGoal(targetPosition.getRotations());
    }

    public Rotation2d getArmPosition() {
        var pos = mCANcoderPivot.getAbsolutePosition().getValueAsDouble();
        return Rotation2d.fromRotations(pos);
    }

    public Rotation2d getArmRelativePos() {
        return Rotation2d.fromRotations(mTalonPivot.getPosition().getValueAsDouble());
    }

    public Rotation2d getArmVelocity() {
        var vel = mCANcoderPivot.getVelocity().getValueAsDouble();
        return Rotation2d.fromRotations(vel);
    }

    @Override
    public void periodic() {
        double voltage;
        if (kLastState != null) {
            voltage = mPPIDController.calculate(getArmPosition().getRotations());
            voltage += mFFController.calculate(getArmPosition().getRotations(), mPPIDController.getSetpoint().velocity);
        } else {
            voltage = 0.0;
        }

        mTalonPivot.setVoltage(voltage);

        // System.out.println("This Periodic is bieng called");
        // Telemetry
        SmartDashboard.putString("KLastState Arm Pivot", kLastState.name());

        SmartDashboard.putNumber("Coral Pivot Pos (rotations)", getArmPosition().getRotations());
        SmartDashboard.putNumber("Coral Pivot Vel (Rotations / sec)", getArmVelocity().getRotations());

        SmartDashboard.putNumber("Coral Arm Pivot * 96 (Roations)", getPivotAngle().getRotations());
        SmartDashboard.putNumber("Coral Arm Pivot Relative (Rotations)", mTalonPivot.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("PID Output (Coral Arm)", mPPIDController.calculate(getArmPosition().getRadians()));
        SmartDashboard.putNumber("FF Output (Coral Arm)", mFFController.calculate(getArmPosition().getRadians(), mPPIDController.getSetpoint().velocity));

        SmartDashboard.putNumber("Coral Pivot Target Pos", mPPIDController.getSetpoint().position);
        SmartDashboard.putNumber("Coral Pivot Target Vel", mPPIDController.getSetpoint().velocity);

        SmartDashboard.putNumber("Coral Pivot Applied Voltage", voltage);
    }

    public static class DefaultCommand extends Command {

        public DefaultCommand() {
            addRequirements(RushinatorPivot.getInstance());
        }

        @Override
        public void execute() {
            if (kLastState == State.kTuck) {
                RushinatorPivot.getInstance().setTargetState(State.kTuck);
            } else {
                RushinatorPivot.getInstance().setTargetState(State.kStowTravel);
            }
        }

    }
}
