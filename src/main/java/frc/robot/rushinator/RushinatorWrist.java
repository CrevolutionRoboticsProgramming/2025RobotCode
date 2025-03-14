package frc.robot.rushinator;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RushinatorWrist extends SubsystemBase {
    public static class Settings {
        static final int kTalonWristID = 98; //TODO
        static final int kCancoderWristID = 99; //TODO

        static final double kG = 0.19; // V
        static final double kS = 0.0; // V / rad
        static final double kV = 0; // V * sec / rad
        static final double kA = 0; // V * sec^2 / rad

        static final Rotation2d kMaxVelocity = Rotation2d.fromDegrees(100);
        static final Rotation2d kMaxAcceleration = Rotation2d.fromDegrees(100);
        static final double kP = 15.0;
        static final double kI = 0.0;
        static final double kD = 0;

    }

    public enum State {
        kGroundWrist(Rotation2d.fromDegrees(180)),
        kScoreWrist(Rotation2d.fromDegrees(90));

        State(Rotation2d pos) {
            this.pos = pos;
        }
        public final Rotation2d pos;
    }


    private final CANcoder mWristCancoder;
    private final TalonFX mWristTalon;
    private final ProfiledPIDController mPPIDController;
    private final SimpleMotorFeedforward mFFController;
    
    public static State kLastState;
    
    public RushinatorWrist() {  
        mWristTalon = new TalonFX(Settings.kTalonWristID);
        mWristCancoder =  new CANcoder(Settings.kCancoderWristID);

        mPPIDController = new ProfiledPIDController(Settings.kP, Settings.kI, Settings.kD, new TrapezoidProfile.Constraints(
                Settings.kMaxVelocity.getRadians(),
                Settings.kMaxAcceleration.getRadians()
        ));
        mPPIDController.setTolerance(2); //degrees of tolerance

        mFFController = new SimpleMotorFeedforward(Settings.kS, Settings.kV, Settings.kA);
    }

    private static RushinatorWrist mInstance;
    public static RushinatorWrist getInstance() {
        if (mInstance == null) {
            mInstance = new RushinatorWrist();
        }
        return mInstance;
    }


    @Override
    public void periodic() {
        double currentAngle = mWristCancoder.getAbsolutePosition().getValueAsDouble();
        double pidOutput = mPPIDController.calculate(currentAngle);
        TrapezoidProfile.State setpoint = mPPIDController.getSetpoint();
        double ffOutput = mFFController.calculate(currentAngle, setpoint.velocity);
        double totalOutputVoltage = pidOutput + ffOutput;
        mWristTalon.setVoltage(totalOutputVoltage);
    }

    public void setTargetState(State targetState) {
        kLastState = targetState;
        setTargetPosition(targetState.pos);
    }

    public void setTargetPosition(Rotation2d targetPosition) {
        mPPIDController.setGoal(targetPosition.getRadians());
    }

    public boolean atSetpoint() {
        return mPPIDController.atGoal();
    }

    public State getCurrentWristState() {
        return kLastState;
    }

    public static class ToggleWristAngle extends Command {
        State mCurrentState = RushinatorWrist.getInstance().getCurrentWristState();
        State mTargetState;
        public ToggleWristAngle(State targetWristState) {
            mTargetState = targetWristState;
            addRequirements(RushinatorWrist.getInstance());
        }

        @Override
        public void initialize() {
            RushinatorWrist.getInstance().setTargetState(mTargetState);
        }

        @Override
        public void execute() {
            
        }

        @Override
        public boolean isFinished() {
            return RushinatorWrist.getInstance().atSetpoint();
        }

        @Override
        public void end(boolean interrupted) {
            
        }

    }
}
