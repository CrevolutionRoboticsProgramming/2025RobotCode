package frc.robot.rushinator;

import static edu.wpi.first.units.Units.Rotation;

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
import frc.robot.rushinator.commands.SetWristState;

public class RushinatorWrist extends SubsystemBase {
    public static class Settings {
        static final int kTalonWristID = 12; 
        static final int kCancoderWristID = 24; 

        public static final double kG = 0.0; // V
        public static final double kS = 0.0; // V / rad
        public static final double kV = 1.6; // V * sec / rad
        public static final double kA = 0.0; // V * sec^2 / rad

        public static final Rotation2d kMaxVelocity = Rotation2d.fromDegrees(10);
        public static final Rotation2d kMaxAcceleration = Rotation2d.fromDegrees(100);
        public static final double kP = 0.05;
        public static final double kI = 0.0;
        public static final double kD = 0.05;

        public static final double kZeroOffset = 0.0; // rotations

    }
// 12.3720703125 Score MId
    public enum State {
        kScoreLeftWrist(Rotation2d.fromRotations(15.62573242 + 23)),
        kScoreRightWrist(Rotation2d.fromRotations(15.62573242 - 23)),
        kScoreMid(Rotation2d.fromRotations(15.62573242)),
        kScoreL1LeftWrist(Rotation2d.fromRotations(10.52197265625 + 23)),
        kScoreL1RightWrist(Rotation2d.fromRotations(10.52197265625 - 23)),
        kScoreL1Mid(Rotation2d.fromRotations(10.52197265625)),
        kHPLeft(Rotation2d.fromRotations(19.2294921875 + 23)),
        kHPRight(Rotation2d.fromRotations(19.2294921875 - 23)),
        kHPMid(Rotation2d.fromRotations(19.2294921875)),
        kGroundLeft(Rotation2d.fromRotations(-4.62841796875 + 23)),
        kGroundRight(Rotation2d.fromRotations(-4.62841796875 - 23)),
        kGroundMid(Rotation2d.fromRotations(-4.62841796875)),
        kTravelLeft(Rotation2d.fromRotations(18.87939453125 + 23)),
        kTravelRight(Rotation2d.fromRotations(18.87939453125 - 23)),
        kTravelMid(Rotation2d.fromRotations(18.87939453125));

        State(Rotation2d pos) {
            this.pos = pos;
        }
        public final Rotation2d pos;
    }


    private final CANcoder mWristCancoder;
    public TalonFX mWristTalon;
    private final ProfiledPIDController mPPIDController;
    private final SimpleMotorFeedforward mFFController;
    
    public static State kLastState;
    
    public RushinatorWrist() {  
        mWristTalon = new TalonFX(Settings.kTalonWristID);
        mWristTalon.getConfigurator().apply(new TalonFXConfiguration().withMotorOutput(new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake)
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

        if (kLastState == null) {
            kLastState = State.kTravelRight;
        }
        mPPIDController.setGoal(kLastState.pos.getRotations());
    }

    private static RushinatorWrist mInstance;
    public static RushinatorWrist getInstance() {
        if (mInstance == null) {
            mInstance = new RushinatorWrist();
        }
        return mInstance;
    }

    public Rotation2d getWristRelativePos() {
        return Rotation2d.fromRotations(mWristTalon.getPosition().getValueAsDouble());
    }


    @Override
    public void periodic() {
        double pidOutput = mPPIDController.calculate(getWristRelativePos().getRotations());
        double ffOutput = mFFController.calculate(getWristRelativePos().getRotations(), mPPIDController.getSetpoint().velocity);
        double totalOutputVoltage = pidOutput + ffOutput;
        mWristTalon.setVoltage(totalOutputVoltage);
        

        // SmartDashboard.putNumber("PID Output", pidOutput);
        // SmartDashboard.putNumber("FF Output", ffOutput);
        // SmartDashboard.putNumber("Output Voltage", totalOutputVoltage);
        SmartDashboard.putString("KLastState Wrist Pivot", kLastState.name());
        SmartDashboard.putNumber("Coral Wrist Current Angle (Rotations)", getCurrentPos().getRotations());
        SmartDashboard.putNumber("Coral Wrist Pivot (Rotations Relavtive)", mWristTalon.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Coral WRist Current Vel", mWristTalon.getVelocity().getValueAsDouble());

        SmartDashboard.putNumber("Coral Wrist Target Pos", mPPIDController.getSetpoint().position);
        SmartDashboard.putNumber("Coral Wrist Target Vel", mPPIDController.getSetpoint().velocity);
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

    public double getMotorOutputVoltage() {
        return mWristTalon.getMotorVoltage().getValueAsDouble();
    }

    public boolean atSetpoint() {
        return mPPIDController.atGoal();
    }

    public State getCurrentWristState() {
        return kLastState;
    }

    public Rotation2d getCurrentPos() {
        return Rotation2d.fromRotations(mWristCancoder.getPosition().getValueAsDouble());
    }

    public Rotation2d getCurrentRelativePos() {
        return Rotation2d.fromRotations(mWristTalon.getPosition().getValueAsDouble());
    }

    public static class DefaultCommand extends Command {
        RushinatorWrist subsystem;
        public DefaultCommand() {
            this.subsystem = RushinatorWrist.getInstance();
            addRequirements(subsystem);
        }

        @Override
        public void execute() {
            // if (kLastState == RushinatorWrist.State.kScoreRightWrist) {
            //     new SetWristState(RushinatorWrist.State.kTravelRight);
            // } else if (kLastState == RushinatorWrist.State.kScoreLeftWrist){
            //     new SetWristState(RushinatorWrist.State.kTravelLeft);
            // } else if (kLastState == RushinatorWrist.State.kTravelLeft){
            //     new SetWristState(RushinatorWrist.State.kTravelLeft);
            // } else if (kLastState == RushinatorWrist.State.kTravelRight) {
            //     new SetWristState(RushinatorWrist.State.kTravelRight);
            // } else {
            //     new SetWristState(RushinatorWrist.State.kTravelRight);
            // }
            new SetWristState(RushinatorWrist.State.kTravelRight);
        }
    }
}
