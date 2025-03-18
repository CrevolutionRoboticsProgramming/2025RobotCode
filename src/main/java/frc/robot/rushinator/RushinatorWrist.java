package frc.robot.rushinator;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RushinatorWrist extends SubsystemBase {
    public static class Settings {
        static final int kTalonWristID = 12; 
        static final int kCancoderWristID = 24; 

        public static final double kG = 0.01; // V
        public static final double kS = 0.0; // V / rad
        public static final double kV = 1.77; // V * sec / rad
        public static final double kA = 0.0; // V * sec^2 / rad

        public static final Rotation2d kMaxVelocity = Rotation2d.fromDegrees(100);
        public static final Rotation2d kMaxAcceleration = Rotation2d.fromDegrees(100);
        public static final double kP = 0.1;
        public static final double kI = 0.0;
        public static final double kD = 0.0;

        public static final double kZeroOffset = 0.10498; // rotations

    }

    public enum State {
        kGroundWrist(Rotation2d.fromRotations(24.08789)),
        kScoreLeftWrist(Rotation2d.fromRotations(0.45)),
        kScoreRightWrits(Rotation2d.fromRotations(-45.13037109375)),
        kHumanPlayer(Rotation2d.fromRotations(-22.8662109375));

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
        mWristTalon.getConfigurator().apply(new TalonFXConfiguration().withMotorOutput(new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive)
                .withNeutralMode(NeutralModeValue.Coast)
        ));

        mWristCancoder =  new CANcoder(Settings.kCancoderWristID);
        mWristCancoder.getConfigurator().apply(new CANcoderConfiguration().withMagnetSensor(new MagnetSensorConfigs().
                withSensorDirection(SensorDirectionValue.Clockwise_Positive).
                withMagnetOffset(Settings.kZeroOffset)
        ));

        mPPIDController = new ProfiledPIDController(Settings.kP, Settings.kI, Settings.kD, new TrapezoidProfile.Constraints(
                Settings.kMaxVelocity.getRadians(),
                Settings.kMaxAcceleration.getRadians()
        ));
        mPPIDController.setTolerance(1); //degrees of tolerance

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
        // mWristTalon.setVoltage(totalOutputVoltage);
        

        // SmartDashboard.putNumber("PID Output", pidOutput);
        // SmartDashboard.putNumber("FF Output", ffOutput);
        // SmartDashboard.putNumber("Output Voltage", totalOutputVoltage);
        SmartDashboard.putNumber("Coral Wrist Current Angle (Rotations)", getCurrentPos().getRotations());
        SmartDashboard.putNumber("Coral Wrist Pivot (Rotations Relavtive)", mWristTalon.getPosition().getValueAsDouble());
    }

    public void setTargetState(State targetState) {
        kLastState = targetState;
        setTargetPosition(targetState.pos);
    }

    public void setTargetPosition(Rotation2d targetPosition) {
        mPPIDController.setGoal(targetPosition.getRadians());
    }

    public void setVoltage(double voltage) {
        mWristTalon.setVoltage(voltage);
    }

    public boolean atSetpoint() {
        return mPPIDController.atGoal();
    }

    public State getCurrentWristState() {
        return kLastState;
    }

    public Rotation2d getCurrentPos() {
        return Rotation2d.fromRotations(mWristTalon.getPosition().getValueAsDouble());
    }

    
    
}
