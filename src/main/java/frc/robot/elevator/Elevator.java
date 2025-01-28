package frc.robot.elevator;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkMaxAlternateEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;



public class Elevator extends SubsystemBase{
    public static class Settings {
        static final int kSparkLeaderID = 22;
        static final int kSparkFollowerID = 23;
        static final boolean kLeftSparkInverted = false;
        static final boolean kRightSparkInverted = false;

        static final int kLowerLimitSwitch = 0;
        static final int kUpperLimitSwitch = 2;

        static final IdleMode kIdleMode = IdleMode.kCoast;

        public static final double kG = 0.01;
        public static final double kS = 0.0;
        public static final double kV = 76.0;
        public static final double kA = 0.0;

        public static final double kP = 0.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;

        public static final double kMaxVelocity = 0.25; //0.15
        public static final double kMaxAcceleration = 0.6;

        public static final double kMaxVoltage = 10.0;
        static final double kSprocketDiameter = Units.inchesToMeters(1.432);

        public static final double kMaxExtension = Units.inchesToMeters(14);
    }

    public static Elevator mInstance;

    public SparkMax mSparkLeader;

    public SparkMax mSparkFollower;
    public final RelativeEncoder mEncoder;
    public final DigitalInput mLowerLimitSwitch;
    public final SparkMaxConfig mLeadSparkConfig, mFollowerSparkConfig;
    public double velocity;

    public final ElevatorFeedforward mFFController;
    public final PIDController mPIDController;

    // public ElevatorState desiredState;

    public Elevator() {
        mSparkLeader = new SparkMax(Settings.kSparkLeaderID, MotorType.kBrushless);
        mLeadSparkConfig = new SparkMaxConfig();
        mLeadSparkConfig.idleMode(Settings.kIdleMode);
        mLeadSparkConfig.inverted(true);
        mSparkLeader.configure(mLeadSparkConfig, null, null);
        
        mSparkFollower = new SparkMax(Settings.kSparkFollowerID, MotorType.kBrushless);
        mFollowerSparkConfig = new SparkMaxConfig();
        mFollowerSparkConfig.follow(mSparkLeader);
        mSparkFollower.configure(mFollowerSparkConfig, null, null);

        mLowerLimitSwitch = new DigitalInput(Settings.kLowerLimitSwitch);
        // mUpperLimitSwitch = new DigitalInput(Settings.kUpperLimitSwitch);

        mEncoder = mSparkLeader.getAlternateEncoder();

        mFFController = new ElevatorFeedforward(Settings.kS, Settings.kG, Settings.kV, Settings.kA);
        mPIDController = new PIDController(Settings.kP, Settings.kI, Settings.kD);
    }

    public static Elevator getInstance() {
        if (mInstance == null) {
            mInstance = new Elevator();
        }
        return mInstance;
    }

    public void setVelocity(double velocity) {
        this.velocity = velocity;
    }

    public boolean getLowerLimitState() {
        return mLowerLimitSwitch.get();
    }

    // public boolean getUpperLimitState() {
    //     return mUpperLimitSwitch.get();
    // }

    public double getPosition() {
        return rotationsToMeters(mEncoder.getPosition());
    }

    public double getVelocity() {
        return rotationsToMeters(mEncoder.getVelocity() / 60.0);
    }

    public void resetEncoder() {
        mEncoder.setPosition(0);
    }

    private double rotationsToMeters(double rotations) {
        return Settings.kSprocketDiameter * Math.PI * rotations;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator Position (m)", getPosition());
        SmartDashboard.putNumber("Elevator Velocity (ms^-1): ", getVelocity());
        SmartDashboard.putBoolean("Elevator Lover lImit Switch Hit?", getLowerLimitState());

        if (getLowerLimitState()) {
            resetEncoder();
        }
        
        // Make Elevator Move
        double speed = mPIDController.calculate(getVelocity(), velocity);
        speed += mFFController.calculate(mFFController.calculate(velocity));

        SmartDashboard.putNumber("mPPIDC + mFFC Output", speed);

        mSparkLeader.setVoltage(speed);

    }
}
