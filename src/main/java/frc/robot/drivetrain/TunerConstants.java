package frc.robot.drivetrain;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.*;
import com.ctre.phoenix6.swerve.*;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.*;
import frc.robot.RobotContainer;
import frc.robot.drivetrain.CommandSwerveDrivetrain;

// Generated by the Tuner X Swerve Project Generator
// https://v6.docs.ctr-electronics.com/en/stable/docs/tuner/tuner-swerve/index.html
public class TunerConstants {
    // Both sets of gains need to be tuned to your individual robot.

    // The steer motor uses any SwerveModule.SteerRequestType control request with the
    // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
    private static final Slot0Configs steerGains = new Slot0Configs()
        .withKP(100).withKI(0).withKD(0.5)
        .withKS(0.1).withKV(1.16).withKA(0)
        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
    // When using closed-loop control, the drive motor uses the control
    // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
    private static final Slot0Configs driveGains = new Slot0Configs()
        .withKP(0.1).withKI(0).withKD(0)
        .withKS(0).withKV(0.124);

    // The closed-loop output type to use for the steer motors;
    // This affects the PID/FF gains for the steer motors
    private static final ClosedLoopOutputType kSteerClosedLoopOutput = ClosedLoopOutputType.Voltage;
    // The closed-loop output type to use for the drive motors;
    // This affects the PID/FF gains for the drive motors
    private static final ClosedLoopOutputType kDriveClosedLoopOutput = ClosedLoopOutputType.Voltage;

    // The type of motor used for the drive motor
    private static final DriveMotorArrangement kDriveMotorType = DriveMotorArrangement.TalonFX_Integrated;
    // The type of motor used for the drive motor
    private static final SteerMotorArrangement kSteerMotorType = SteerMotorArrangement.TalonFX_Integrated;

    // The remote sensor feedback type to use for the steer motors;
    // When not Pro-licensed, Fused*/Sync* automatically fall back to Remote*
    private static final SteerFeedbackType kSteerFeedbackType = SteerFeedbackType.FusedCANcoder;

    // The stator current at which the wheels start to slip;
    // This needs to be tuned to your individual robot
    private static final Current kSlipCurrent = Amps.of(120.0);

    // Initial configs for the drive and steer motors and the azimuth encoder; these cannot be null.
    // Some configs will be overwritten; check the `with*InitialConfigs()` API documentation.
    private static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration();
    private static final TalonFXConfiguration steerInitialConfigs = new TalonFXConfiguration()
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                // Swerve azimuth does not require much torque output, so we can set a relatively low
                // stator current limit to help avoid brownouts without impacting performance.
                .withStatorCurrentLimit(Amps.of(60))
                .withStatorCurrentLimitEnable(true)
        );
    private static final CANcoderConfiguration encoderInitialConfigs = new CANcoderConfiguration();
    // Configs for the Pigeon 2; leave this null to skip applying Pigeon 2 configs
    private static final Pigeon2Configuration pigeonConfigs = null;

    // CAN bus that the devices are located on;
    // All swerve devices must share the same CAN bus
    public static final CANBus kCANBus = new CANBus("Canivore", "./logs/example.hoot");

    // Theoretical free speed (m/s) at 12 V applied output;
    // This needs to be tuned to your individual robot
    public static final LinearVelocity kSpeedAt12Volts = MetersPerSecond.of(5.15);

    // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
    // This may need to be tuned to your individual robot
    private static final double kCoupleRatio = 4.909090909090909;

    private static final double kDriveGearRatio = 6.2009569377990434;
    private static final double kSteerGearRatio = 12.1;
    private static final Distance kWheelRadius = Inches.of(2);

    private static final boolean kInvertLeftSide = false;
    private static final boolean kInvertRightSide = true;

    static final int kPigeonId = 18;

    // These are only used for simulation
    private static final MomentOfInertia kSteerInertia = KilogramSquareMeters.of(0.01);
    private static final MomentOfInertia kDriveInertia = KilogramSquareMeters.of(0.01);
    // Simulated voltage necessary to overcome friction
    private static final Voltage kSteerFrictionVoltage = Volts.of(0.2);
    private static final Voltage kDriveFrictionVoltage = Volts.of(0.2);

    public static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
            .withCANBusName(kCANBus.getName())
            .withPigeon2Id(kPigeonId)
            .withPigeon2Configs(pigeonConfigs);

    private static final SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> ConstantCreator =
        new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
            .withDriveMotorGearRatio(kDriveGearRatio)
            .withSteerMotorGearRatio(kSteerGearRatio)
            .withCouplingGearRatio(kCoupleRatio)
            .withWheelRadius(kWheelRadius)
            .withSteerMotorGains(steerGains)
            .withDriveMotorGains(driveGains)
            .withSteerMotorClosedLoopOutput(kSteerClosedLoopOutput)
            .withDriveMotorClosedLoopOutput(kDriveClosedLoopOutput)
            .withSlipCurrent(kSlipCurrent)
            .withSpeedAt12Volts(kSpeedAt12Volts)
            .withDriveMotorType(kDriveMotorType)
            .withSteerMotorType(kSteerMotorType)
            .withFeedbackSource(kSteerFeedbackType)
            .withDriveMotorInitialConfigs(driveInitialConfigs)
            .withSteerMotorInitialConfigs(steerInitialConfigs)
            .withEncoderInitialConfigs(encoderInitialConfigs)
            .withSteerInertia(kSteerInertia)
            .withDriveInertia(kDriveInertia)
            .withSteerFrictionVoltage(kSteerFrictionVoltage)
            .withDriveFrictionVoltage(kDriveFrictionVoltage);


    // Front Left
    private static final int kFrontLeftDriveMotorId = 1;
    static final int kFrontLeftSteerMotorId = 5;
    private static final int kFrontLeftEncoderId = 19;
    private static final Angle kFrontLeftEncoderOffset = Rotations.of(0.387939453125);
    private static final boolean kFrontLeftSteerMotorInverted = false;
    private static final boolean kFrontLeftEncoderInverted = false;

    private static final Distance kFrontLeftXPos = Inches.of(10.5);
    private static final Distance kFrontLeftYPos = Inches.of(10.5);

    // Front Right
    private static final int kFrontRightDriveMotorId = 2;
    static final int kFrontRightSteerMotorId = 6;
    private static final int kFrontRightEncoderId = 20;
    private static final Angle kFrontRightEncoderOffset = Rotations.of(0.10791015625);
    private static final boolean kFrontRightSteerMotorInverted = false;
    private static final boolean kFrontRightEncoderInverted = false;

    private static final Distance kFrontRightXPos = Inches.of(10.5);
    private static final Distance kFrontRightYPos = Inches.of(-10.5);

    // Back Left
    private static final int kBackLeftDriveMotorId = 3;
    static final int kBackLeftSteerMotorId = 7;
    private static final int kBackLeftEncoderId = 21;
    private static final Angle kBackLeftEncoderOffset = Rotations.of(0.08154296875);
    private static final boolean kBackLeftSteerMotorInverted = false;
    private static final boolean kBackLeftEncoderInverted = false;

    private static final Distance kBackLeftXPos = Inches.of(-10.5);
    private static final Distance kBackLeftYPos = Inches.of(10.5);

    // Back Right
    private static final int kBackRightDriveMotorId = 4;
    static final int kBackRightSteerMotorId = 8;
    private static final int kBackRightEncoderId = 22;
    private static final Angle kBackRightEncoderOffset = Rotations.of(0.268310546875);
    private static final boolean kBackRightSteerMotorInverted = false;
    private static final boolean kBackRightEncoderInverted = false;

    private static final Distance kBackRightXPos = Inches.of(-10.5);
    private static final Distance kBackRightYPos = Inches.of(-10.5);


    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontLeft =
        ConstantCreator.createModuleConstants(
            kFrontLeftSteerMotorId, kFrontLeftDriveMotorId, kFrontLeftEncoderId, kFrontLeftEncoderOffset,
            kFrontLeftXPos, kFrontLeftYPos, kInvertLeftSide, kFrontLeftSteerMotorInverted, kFrontLeftEncoderInverted
        );
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontRight =
        ConstantCreator.createModuleConstants(
            kFrontRightSteerMotorId, kFrontRightDriveMotorId, kFrontRightEncoderId, kFrontRightEncoderOffset,
            kFrontRightXPos, kFrontRightYPos, kInvertRightSide, kFrontRightSteerMotorInverted, kFrontRightEncoderInverted
        );
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackLeft =
        ConstantCreator.createModuleConstants(
            kBackLeftSteerMotorId, kBackLeftDriveMotorId, kBackLeftEncoderId, kBackLeftEncoderOffset,
            kBackLeftXPos, kBackLeftYPos, kInvertLeftSide, kBackLeftSteerMotorInverted, kBackLeftEncoderInverted
        );
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackRight =
        ConstantCreator.createModuleConstants(
            kBackRightSteerMotorId, kBackRightDriveMotorId, kBackRightEncoderId, kBackRightEncoderOffset,
            kBackRightXPos, kBackRightYPos, kInvertRightSide, kBackRightSteerMotorInverted, kBackRightEncoderInverted
        );

    /**
     * Creates a CommandSwerveDrivetrain instance.
     * This should only be called once in your robot program,.
     */
    public static CommandSwerveDrivetrain createDrivetrain() {
        return new CommandSwerveDrivetrain(
            DrivetrainConstants, FrontLeft, FrontRight, BackLeft, BackRight
        );
    }

    // public static final SwerveModulePosition[] K_SWERVE_MODULE_POSITIONS = new SwerveModulePosition[]{
    //     new SwerveModulePosition(kFrontLeftXPos, Rotation2d.fromDegrees(CommandSwerveDrivetrain.FrontLeftSteerMotor.getPosition().getValueAsDouble())),
    //     new SwerveModulePosition(kFrontRightXPos, Rotation2d.fromDegrees(CommandSwerveDrivetrain.FrontRightSteerMotor.getPosition().getValueAsDouble())),
    //     new SwerveModulePosition(kBackLeftXPos, Rotation2d.fromDegrees(CommandSwerveDrivetrain.BackLeftSteerMotor.getPosition().getValueAsDouble())),
    //     new SwerveModulePosition(kBackRightXPos, Rotation2d.fromDegrees(CommandSwerveDrivetrain.BackRightSteerMotor.getPosition().getValueAsDouble()))
    // };

    /**
     * Swerve Drive class utilizing CTR Electronics' Phoenix 6 API with the selected device types.
     */
    public static class TunerSwerveDrivetrain extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> {
        /**
         * Constructs a CTRE SwerveDrivetrain using the specified constants.
         * <p>
         * This constructs the underlying hardware devices, so users should not construct
         * the devices themselves. If they need the devices, they can access them through
         * getters in the classes.
         *
         * @param drivetrainConstants   Drivetrain-wide constants for the swerve drive
         * @param modules               Constants for each specific module
         */
        public TunerSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules
        ) {
            super(
                TalonFX::new, TalonFX::new, CANcoder::new,
                drivetrainConstants, modules
            );
        }

        /**
         * Constructs a CTRE SwerveDrivetrain using the specified constants.
         * <p>
         * This constructs the underlying hardware devices, so users should not construct
         * the devices themselves. If they need the devices, they can access them through
         * getters in the classes.
         *
         * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
         * @param odometryUpdateFrequency The frequency to run the odometry loop. If
         *                                unspecified or set to 0 Hz, this is 250 Hz on
         *                                CAN FD, and 100 Hz on CAN 2.0.
         * @param modules                 Constants for each specific module
         */
        public TunerSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            SwerveModuleConstants<?, ?, ?>... modules
        ) {
            super(
                TalonFX::new, TalonFX::new, CANcoder::new,
                drivetrainConstants, odometryUpdateFrequency, modules
            );
        }

        /**
         * Constructs a CTRE SwerveDrivetrain using the specified constants.
         * <p>
         * This constructs the underlying hardware devices, so users should not construct
         * the devices themselves. If they need the devices, they can access them through
         * getters in the classes.
         *
         * @param drivetrainConstants       Drivetrain-wide constants for the swerve drive
         * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
         *                                  unspecified or set to 0 Hz, this is 250 Hz on
         *                                  CAN FD, and 100 Hz on CAN 2.0.
         * @param odometryStandardDeviation The standard deviation for odometry calculation
         *                                  in the form [x, y, theta]ᵀ, with units in meters
         *                                  and radians
         * @param visionStandardDeviation   The standard deviation for vision calculation
         *                                  in the form [x, y, theta]ᵀ, with units in meters
         *                                  and radians
         * @param modules                   Constants for each specific module
         */
        public TunerSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            Matrix<N3, N1> odometryStandardDeviation,
            Matrix<N3, N1> visionStandardDeviation,
            SwerveModuleConstants<?, ?, ?>... modules
        ) {
            super(
                TalonFX::new, TalonFX::new, CANcoder::new,
                drivetrainConstants, odometryUpdateFrequency,
                odometryStandardDeviation, visionStandardDeviation, modules
            );
        }
    }

    public static class DriveCommandsConstants{

        //TODO: tune these constants correctly so robot doesn't go haywire
        public static final Distance TRANSLATION_TOLERANCE = Inches.of(0.5);
        public static final Angle THETA_TOLERANCE = Degrees.of(1.0);

        public static final LinearVelocity MAX_DRIVE_POSE_TRANS_VELOCITY = MetersPerSecond.of(RobotContainer.kMaxVelocity);
        public static final LinearAcceleration MAX_DRIVE_POSE_TRANS_ACCELERATION = MetersPerSecondPerSecond.of(3.0);

        public static final AngularVelocity MAX_DRIVE_POSE_ANGULAR_VELOCITY = RadiansPerSecond.of(RobotContainer.kMaxAngularVelocity);
        public static final AngularAcceleration MAX_DRIVE_POSE_ANGULAR_ACCELERATION = RadiansPerSecondPerSecond.of(3 * Math.PI);

        public static final double THETA_kP = 0.0;
        public static final double THETA_kI = 0.0;
        public static final double THETA_kD = 0.0;
    
        public static final double X_kP = 0.0;
        public static final double X_kI = 0.0;
        public static final double X_kD = 0.0;
    
        public static final double Y_kP = 0.0;
        public static final double Y_kI = 0.0;
        public static final double Y_kD = 0.0;

        public static final TrapezoidProfile.Constraints DEFAULT_XY_CONSTRAINTS = 
            new TrapezoidProfile.Constraints(
                MAX_DRIVE_POSE_TRANS_VELOCITY.in(MetersPerSecond),
                MAX_DRIVE_POSE_TRANS_ACCELERATION.in(MetersPerSecondPerSecond));
    
        public static final TrapezoidProfile.Constraints DEFAULT_OMEGA_CONSTRAINTS = 
            new TrapezoidProfile.Constraints(
                MAX_DRIVE_POSE_ANGULAR_VELOCITY.in(RadiansPerSecond),
                MAX_DRIVE_POSE_ANGULAR_ACCELERATION.in(RadiansPerSecondPerSecond));
        
            
    } 
}


// import static edu.wpi.first.units.Units.*;

// import java.util.function.Supplier;

// import com.ctre.phoenix6.CANBus;
// import com.ctre.phoenix6.configs.*;
// import com.ctre.phoenix6.hardware.*;
// import com.ctre.phoenix6.signals.*;
// import com.ctre.phoenix6.swerve.*;
// import com.ctre.phoenix6.swerve.SwerveModuleConstants.*;

// import edu.wpi.first.math.Matrix;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.kinematics.SwerveModulePosition;
// import edu.wpi.first.math.numbers.N1;
// import edu.wpi.first.math.numbers.N3;
// import edu.wpi.first.units.measure.*;
// import edu.wpi.first.wpilibj.motorcontrol.Talon;
// import frc.crevolib.math.Conversions;
// import frc.robot.drivetrain.CommandSwerveDrivetrain;
// import frc.robot.vision.Vision;
// import frc.robot.vision.VisionConfig;

// import com.ctre.phoenix6.swerve.SwerveDrivetrain.DeviceConstructor.*;

// // Generated by the Tuner X Swerve Project Generator
// // https://v6.docs.ctr-electronics.com/en/stable/docs/tuner/tuner-swerve/index.html
// public class TunerConstants {
//     // Both sets of gains need to be tuned to your individual robot.

//     // The steer motor uses any SwerveModule.SteerRequestType control request with the
//     // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput

//     /* ORIGINAL CONSTNATS */
//     // private static final Slot0Configs steerGains = new Slot0Configs()
//     //     .withKP(100).withKI(0).withKD(0.5)
//     //     .withKS(0.1).withKV(1.16).withKA(0)
//     //     .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
//     private static final Slot0Configs steerGains = new Slot0Configs()
//         .withKP(100).withKI(0.00).withKD(0.5)
//         .withKS(0.29).withKV(2.66).withKA(0)
//         .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
//     // When using closed-loop control, the drive motor uses the control
//     // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput

//     /*ORIGINAL CONSTANTS */
//     // private static final Slot0Configs driveGains = new Slot0Configs()
//     //     .withKP(0.1).withKI(0).withKD(0)
//     //     .withKS(0).withKV(0.124);
//     private static final Slot0Configs driveGains = new Slot0Configs()
//         .withKP(0.1).withKI(0).withKD(0.0)
//         .withKS(0.0).withKV(0.124);

//     // The closed-loop output type to use for the steer motors;
//     // This affects the PID/FF gains for the steer motors
//     private static final ClosedLoopOutputType kSteerClosedLoopOutput = ClosedLoopOutputType.Voltage;
//     // The closed-loop output type to use for the drive motors;
//     // This affects the PID/FF gains for the drive motors
//     private static final ClosedLoopOutputType kDriveClosedLoopOutput = ClosedLoopOutputType.Voltage;

//     // The type of motor used for the drive motor
//     private static final DriveMotorArrangement kDriveMotorType = DriveMotorArrangement.TalonFX_Integrated;
//     // The type of motor used for the drive motor
//     private static final SteerMotorArrangement kSteerMotorType = SteerMotorArrangement.TalonFX_Integrated;

//     // The remote sensor feedback type to use for the steer motors;
//     // When not Pro-licensed, Fused*/Sync* automatically fall back to Remote*
//     private static final SteerFeedbackType kSteerFeedbackType = SteerFeedbackType.FusedCANcoder;

//     // The stator current at which the wheels start to slip;
//     // This needs to be tuned to your individual robot
//     private static final Current kSlipCurrent = Amps.of(120.0);

//     // Initial configs for the drive and steer motors and the azimuth encoder; these cannot be null.
//     // Some configs will be overwritten; check the `with*InitialConfigs()` API documentation.
//     private static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration();
//     private static final TalonFXConfiguration steerInitialConfigs = new TalonFXConfiguration()
//         .withCurrentLimits(
//             new CurrentLimitsConfigs()
//                 // Swerve azimuth does not require much torque output, so we can set a relatively low
//                 // stator current limit to help avoid brownouts without impacting performance.
//                 .withStatorCurrentLimit(Amps.of(60))
//                 .withStatorCurrentLimitEnable(true)
//         );
//     private static final CANcoderConfiguration encoderInitialConfigs = new CANcoderConfiguration();
//     // Configs for the Pigeon 2; leave this null to skip applying Pigeon 2 configs
//     private static final Pigeon2Configuration pigeonConfigs = null;

//     // CAN bus that the devices are located on;
//     // All swerve devices must share the same CAN bus
//     public static final CANBus kCANBus = new CANBus("Canivore", "./logs/example.hoot");

//     // Theoretical free speed (m/s) at 12 V applied output;
//     // This needs to be tuned to your individual robot
//     public static final LinearVelocity kSpeedAt12Volts = MetersPerSecond.of(5.18);

//     // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
//     // This may need to be tuned to your individual robot
//     private static final double kCoupleRatio = 4.909090909090909;

//     private static final double kDriveGearRatio = 6.2009569377990434;
//     private static final double kSteerGearRatio = 12.1;
//     private static final Distance kWheelRadius = Inches.of(4);

//     private static final boolean kInvertLeftSide = false;
//     private static final boolean kInvertRightSide = true;

//     public static final int kPigeonId = 18;

//     // These are only used for simulation
//     private static final MomentOfInertia kSteerInertia = KilogramSquareMeters.of(0.01);
//     private static final MomentOfInertia kDriveInertia = KilogramSquareMeters.of(0.01);
//     // Simulated voltage necessary to overcome friction
//     private static final Voltage kSteerFrictionVoltage = Volts.of(0.2);
//     private static final Voltage kDriveFrictionVoltage = Volts.of(0.2);

//     public static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
//             .withCANBusName(kCANBus.getName())
//             .withPigeon2Id(kPigeonId)
//             .withPigeon2Configs(pigeonConfigs);

//     private static final SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> ConstantCreator =
//         new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
//             .withDriveMotorGearRatio(kDriveGearRatio)
//             .withSteerMotorGearRatio(kSteerGearRatio)
//             .withCouplingGearRatio(kCoupleRatio)
//             .withWheelRadius(kWheelRadius)
//             .withSteerMotorGains(steerGains)
//             .withDriveMotorGains(driveGains)
//             .withSteerMotorClosedLoopOutput(kSteerClosedLoopOutput)
//             .withDriveMotorClosedLoopOutput(kDriveClosedLoopOutput)
//             .withSlipCurrent(kSlipCurrent)
//             .withSpeedAt12Volts(kSpeedAt12Volts)
//             .withDriveMotorType(kDriveMotorType)
//             .withSteerMotorType(kSteerMotorType)
//             .withFeedbackSource(kSteerFeedbackType)
//             .withDriveMotorInitialConfigs(driveInitialConfigs)
//             .withSteerMotorInitialConfigs(steerInitialConfigs)
//             .withEncoderInitialConfigs(encoderInitialConfigs)
//             .withSteerInertia(kSteerInertia)
//             .withDriveInertia(kDriveInertia)
//             .withSteerFrictionVoltage(kSteerFrictionVoltage)
//             .withDriveFrictionVoltage(kDriveFrictionVoltage);


//     // Front Left
//     private static final int kFrontLeftDriveMotorId = 1;
//     public static final int kFrontLeftSteerMotorId = 5;
//     private static final int kFrontLeftEncoderId = 19;
//     private static final Angle kFrontLeftEncoderOffset = Rotations.of(0.384765625);
//     // private static final Angle kFrontLeftEncoderOffset = Rotations.of(-6.500732421875001);
//     private static final boolean kFrontLeftSteerMotorInverted = false;
//     private static final boolean kFrontLeftEncoderInverted = false;

//     private static final Distance kFrontLeftXPos = Inches.of(10.5);
//     private static final Distance kFrontLeftYPos = Inches.of(10.5);
//     // private static final Distance kFrontLeftYPos = Inches.of(14.845);

//     // Front Right
//     private static final int kFrontRightDriveMotorId = 2;
//     public static final int kFrontRightSteerMotorId = 6;
//     private static final int kFrontRightEncoderId = 20;
//     private static final Angle kFrontRightEncoderOffset = Rotations.of(0.110107421875);
//     // private static final Angle kFrontRightEncoderOffset = Rotations.of(-2.499267578125);
//     private static final boolean kFrontRightSteerMotorInverted = false;
//     private static final boolean kFrontRightEncoderInverted = false;

//     private static final Distance kFrontRightXPos = Inches.of(10.5);
//     private static final Distance kFrontRightYPos = Inches.of(-10.5);
//     // private static final Distance kFrontRightYPos = Inches.of(-14.845);

//     // Back Left
//     private static final int kBackLeftDriveMotorId = 3;
//     public static final int kBackLeftSteerMotorId = 7;
//     private static final int kBackLeftEncoderId = 21;
//     private static final Angle kBackLeftEncoderOffset = Rotations.of(0.086181640625);
//     // private static final Angle kBackLeftEncoderOffset = Rotations.of(-3.5009765625);
//     private static final boolean kBackLeftSteerMotorInverted = false;
//     private static final boolean kBackLeftEncoderInverted = false;

//     private static final Distance kBackLeftXPos = Inches.of(-10.5);
//     private static final Distance kBackLeftYPos = Inches.of(10.5);
//     // private static final Distance kBackLeftYPos = Inches.of(14.845);

//     // Back Right
//     private static final int kBackRightDriveMotorId = 4;
//     public static final int kBackRightSteerMotorId = 8;
//     private static final int kBackRightEncoderId = 22;
//     private static final Angle kBackRightEncoderOffset = Rotations.of(0.274658203125);
//     // private static final Angle kBackRightEncoderOffset = Rotations.of(-1.9997558593750002);
//     private static final boolean kBackRightSteerMotorInverted = false;
//     private static final boolean kBackRightEncoderInverted = false;

//     private static final Distance kBackRightXPos = Inches.of(-10.5);
//     private static final Distance kBackRightYPos = Inches.of(-10.5);
//     // private static final Distance kBackRightYPos = Inches.of(-14.845);


//     public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontLeft =
//         ConstantCreator.createModuleConstants(
//             kFrontLeftSteerMotorId, kFrontLeftDriveMotorId, kFrontLeftEncoderId, kFrontLeftEncoderOffset,
//             kFrontLeftXPos, kFrontLeftYPos, kInvertLeftSide, kFrontLeftSteerMotorInverted, kFrontLeftEncoderInverted
//         );
//     public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontRight =
//         ConstantCreator.createModuleConstants(
//             kFrontRightSteerMotorId, kFrontRightDriveMotorId, kFrontRightEncoderId, kFrontRightEncoderOffset,
//             kFrontRightXPos, kFrontRightYPos, kInvertRightSide, kFrontRightSteerMotorInverted, kFrontRightEncoderInverted
//         );
//     public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackLeft =
//         ConstantCreator.createModuleConstants(
//             kBackLeftSteerMotorId, kBackLeftDriveMotorId, kBackLeftEncoderId, kBackLeftEncoderOffset,
//             kBackLeftXPos, kBackLeftYPos, kInvertLeftSide, kBackLeftSteerMotorInverted, kBackLeftEncoderInverted
//         );
//     public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackRight =
//         ConstantCreator.createModuleConstants(
//             kBackRightSteerMotorId, kBackRightDriveMotorId, kBackRightEncoderId, kBackRightEncoderOffset,
//             kBackRightXPos, kBackRightYPos, kInvertRightSide, kBackRightSteerMotorInverted, kBackRightEncoderInverted
//         );


//     /**
//      * Creates a CommandSwerveDrivetrain instance.
//      * This should only be called once in your robot program,.
//      */
//     public static CommandSwerveDrivetrain createDrivetrain() {
//         return new CommandSwerveDrivetrain(
//             DrivetrainConstants, FrontLeft, FrontRight, BackLeft, BackRight
//         );
//     }

//     public static final SwerveModulePosition[] K_SWERVE_MODULE_POSITIONS = new SwerveModulePosition[]{
//         new SwerveModulePosition(kFrontLeftXPos, Rotation2d.fromDegrees(CommandSwerveDrivetrain.FrontLeftSteerMotor.getPosition().getValueAsDouble())),
//         new SwerveModulePosition(kFrontRightXPos, Rotation2d.fromDegrees(CommandSwerveDrivetrain.FrontRightSteerMotor.getPosition().getValueAsDouble())),
//         new SwerveModulePosition(kBackLeftXPos, Rotation2d.fromDegrees(CommandSwerveDrivetrain.BackLeftSteerMotor.getPosition().getValueAsDouble())),
//         new SwerveModulePosition(kBackRightXPos, Rotation2d.fromDegrees(CommandSwerveDrivetrain.BackRightSteerMotor.getPosition().getValueAsDouble()))
//     };

//     /**
//      * Swerve Drive class utilizing CTR Electronics' Phoenix 6 API with the selected device types.
//      */
//     public static class TunerSwerveDrivetrain extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> {
//         /**
//          * Constructs a CTRE SwerveDrivetrain using the specified constants.
//          * <p>
//          * This constructs the underlying hardware devices, so users should not construct
//          * the devices themselves. If they need the devices, they can access them through
//          * getters in the classes.
//          *
//          * @param drivetrainConstants   Drivetrain-wide constants for the swerve drive
//          * @param modules               Constants for each specific module
//          */
//         public TunerSwerveDrivetrain(
//             SwerveDrivetrainConstants drivetrainConstants,
//             SwerveModuleConstants<?, ?, ?>... modules
//         ) {
//             super(
//                 TalonFX::new, TalonFX::new, CANcoder::new,
//                 drivetrainConstants, modules
//             );
//         }

//         /**
//          * Constructs a CTRE SwerveDrivetrain using the specified constants.
//          * <p>
//          * This constructs the underlying hardware devices, so users should not construct
//          * the devices themselves. If they need the devices, they can access them through
//          * getters in the classes.
//          *
//          * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
//          * @param odometryUpdateFrequency The frequency to run the odometry loop. If
//          *                                unspecified or set to 0 Hz, this is 250 Hz on
//          *                                CAN FD, and 100 Hz on CAN 2.0.
//          * @param modules                 Constants for each specific module
//          */
//         public TunerSwerveDrivetrain(
//             SwerveDrivetrainConstants drivetrainConstants,
//             double odometryUpdateFrequency,
//             SwerveModuleConstants<?, ?, ?>... modules
//         ) {
//             super(
//                 TalonFX::new, TalonFX::new, CANcoder::new,
//                 drivetrainConstants, odometryUpdateFrequency, modules
//             );
//         }

//         /**
//          * Constructs a CTRE SwerveDrivetrain using the specified constants.
//          * <p>
//          * This constructs the underlying hardware devices, so users should not construct
//          * the devices themselves. If they need the devices, they can access them through
//          * getters in the classes.
//          *
//          * @param drivetrainConstants       Drivetrain-wide constants for the swerve drive
//          * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
//          *                                  unspecified or set to 0 Hz, this is 250 Hz on
//          *                                  CAN FD, and 100 Hz on CAN 2.0.
//          * @param odometryStandardDeviation The standard deviation for odometry calculation
//          *                                  in the form [x, y, theta]ᵀ, with units in meters
//          *                                  and radians
//          * @param visionStandardDeviation   The standard deviation for vision calculation
//          *                                  in the form [x, y, theta]ᵀ, with units in meters
//          *                                  and radians
//          * @param modules                   Constants for each specific module
//          */
//         public TunerSwerveDrivetrain(
//             SwerveDrivetrainConstants drivetrainConstants,
//             double odometryUpdateFrequency,
//             Matrix<N3, N1> odometryStandardDeviation,
//             Matrix<N3, N1> visionStandardDeviation,
//             SwerveModuleConstants<?, ?, ?>... modules
//         ) {
//             super(
//                 TalonFX::new, TalonFX::new, CANcoder::new,
//                 drivetrainConstants, odometryUpdateFrequency,
//                 odometryStandardDeviation, visionStandardDeviation, modules
//             );
//         }
//     }
// }
