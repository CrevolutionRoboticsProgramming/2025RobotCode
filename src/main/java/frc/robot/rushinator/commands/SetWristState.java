package frc.robot.rushinator.commands;

import java.lang.invoke.VolatileCallSite;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.rushinator.RushinatorPivot;
import frc.robot.rushinator.RushinatorWrist;
import frc.robot.rushinator.RushinatorWrist.Settings;
import frc.robot.rushinator.RushinatorWrist.State;

// public class SetWristState extends Command{
//     State mCurrentState = RushinatorWrist.getInstance().getCurrentWristState();
//     State mTargetState;
//     private final ProfiledPIDController mPPIDController;
//     private final SimpleMotorFeedforward mFFController;
//     private final RushinatorWrist mRushWrist;

//     public SetWristState(State targetWristState) {
//         mTargetState = targetWristState;

//          mPPIDController = new ProfiledPIDController(Settings.kP, Settings.kI, Settings.kD, new TrapezoidProfile.Constraints(
//                 Settings.kMaxVelocity.getRadians(),
//                 Settings.kMaxAcceleration.getRadians()
//         ));
//         mPPIDController.setTolerance(1); //degrees of tolerance

//         mFFController = new SimpleMotorFeedforward(Settings.kS, Settings.kV, Settings.kA);

//         mRushWrist = RushinatorWrist.getInstance();
//         addRequirements(RushinatorWrist.getInstance());
//     }

//     @Override
//     public void initialize() {
//         // mPPIDController.setGoal(mTargetState.pos.getRadians());
//         SmartDashboard.putNumber("SetPoint", mTargetState.pos.getRotations());
//         Rotation2d offSet = Rotation2d.fromRotations(-89.0 * RushinatorPivot.getInstance().getArmPosition().getRotations() + 13.3);
//         mPPIDController.setGoal(mTargetState.pos.getRadians() + offSet.getRadians());
//     }

//     @Override
//     public void execute() {

//         var voltage = mPPIDController.calculate(mRushWrist.getCurrentPos().getRadians());
//         voltage += mFFController.calculate(mPPIDController.getSetpoint().velocity);
//         // voltage += mFFController.calculate(mRushWrist.getCurrentPos().getRadians(), mPPIDController.getSetpoint().velocity);
//         mRushWrist.setVoltage(voltage);

//         SmartDashboard.putNumber("PID Output", mPPIDController.calculate(mRushWrist.getCurrentPos().getRadians()));
//         SmartDashboard.putNumber("FF Output", mFFController.calculate(mRushWrist.getCurrentPos().getRadians(), mPPIDController.getSetpoint().velocity));
//         SmartDashboard.putNumber("Output Voltage", voltage);
//     }

//     @Override
//     public boolean isFinished() {
//         return false;
//     }

//     @Override
//     public void end(boolean interrupted) {
//         // mRushWrist.setVoltage(0);
//     }
// }

public class SetWristState extends Command {
    private final RushinatorWrist mRushinatorWrist;
    private final RushinatorPivot mRushinatorPivot;
    private final RushinatorWrist.State mTargetState;
    private final RushinatorPivot.State mTargetArmState;
    private double setpointAngle;

    public SetWristState(RushinatorWrist.State targetWristState) {
        mRushinatorWrist = RushinatorWrist.getInstance();

        mTargetState = targetWristState;
        mTargetArmState = null;

        mRushinatorPivot = RushinatorPivot.getInstance();

        addRequirements(mRushinatorWrist);
    }

    public SetWristState(RushinatorWrist.State targetWristState, RushinatorPivot.State targetArmState) {
        mRushinatorWrist = RushinatorWrist.getInstance();

        mTargetState = targetWristState;
        mTargetArmState= targetArmState;

        mRushinatorPivot = RushinatorPivot.getInstance();

        addRequirements(mRushinatorWrist);
    }

    @Override
    public void initialize() {
        /*neW Version of grabbing pos (Should work?) */
        // setpointAngle = mTargetState.pos.getRadians() - (RushinatorPivot.State.kStow.pos.getRadians() - mTargetArmState.pos.getRadians());
        setpointAngle = mTargetState.pos.getRotations();
        mRushinatorWrist.setTargetState(mTargetState);
        
        SmartDashboard.putNumber("Current Arm Pivot (Radians)", setpointAngle);
        SmartDashboard.putNumber("Actual Setpoint (Rotations)", Units.radiansToRotations(setpointAngle));
        
    }

    @Override
    public void execute() {
        double Voltage;

        if (mTargetState.pos.getRotations() > mRushinatorWrist.getCurrentRelativePos().getRotations()) {
            Voltage = 3.0;
        } else {
            Voltage = -3.0;
        }

        var error = setpointAngle - mRushinatorWrist.getCurrentRelativePos().getRotations();
        if (-0.75 <= error && error <= 0.75) {
            Voltage = 0.0;
        }
        
        mRushinatorWrist.setVoltage(Voltage);

        SmartDashboard.putNumber("Error (radians)", error);
        SmartDashboard.putNumber("Total Voltage", Voltage);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        mRushinatorWrist.setVoltage(0);
    }


// public class SetWristState extends Command {
//     private final RushinatorWrist mRushinatorWrist;
//     private final RushinatorPivot mRushinatorPivot;
//     private final ProfiledPIDController mPPIDController;
//     private final SimpleMotorFeedforward mFFController;
//     private final RushinatorWrist.State mTargetState;
//     private final RushinatorPivot.State mTargetArmState;
//     private double setpointAngle;

//     public SetWristState(RushinatorWrist.State targetWristState) {
//         mRushinatorWrist = RushinatorWrist.getInstance();

//         mPPIDController = new ProfiledPIDController(RushinatorWrist.Settings.kP, RushinatorWrist.Settings.kI, RushinatorWrist.Settings.kD, new TrapezoidProfile.Constraints(
//                             RushinatorWrist.Settings.kMaxVelocity.getRadians(),
//                             RushinatorWrist.Settings.kMaxAcceleration.getRadians()
//                     ));

//         mTargetState = targetWristState;
//         mTargetArmState = null;

//         mFFController = new SimpleMotorFeedforward(Settings.kS, Settings.kV, Settings.kA);

//         mRushinatorPivot = RushinatorPivot.getInstance();

//         addRequirements(mRushinatorWrist);
//     }

//     public SetWristState(RushinatorWrist.State targetWristState, RushinatorPivot.State targetArmState) {
//         mRushinatorWrist = RushinatorWrist.getInstance();

//         mPPIDController = new ProfiledPIDController(RushinatorWrist.Settings.kP, RushinatorWrist.Settings.kI, RushinatorWrist.Settings.kD, new TrapezoidProfile.Constraints(
//                             RushinatorWrist.Settings.kMaxVelocity.getRadians(),
//                             RushinatorWrist.Settings.kMaxAcceleration.getRadians()
//                     ));

//         mTargetState = targetWristState;
//         mTargetArmState= targetArmState;

//         mFFController = new SimpleMotorFeedforward(Settings.kS, Settings.kV, Settings.kA);

//         mRushinatorPivot = RushinatorPivot.getInstance();

//         addRequirements(mRushinatorWrist);
//     }

//     @Override
//     public void initialize() {
//         // setpointAngle = mTargetState.pos.getRadians() - mRushinatorWrist.getCurrentRelativePos().getRadians();
//         /*neW Version of grabbing pos (Should work?) */
//         // setpointAngle = mTargetState.pos.getRadians() - (RushinatorPivot.State.kStow.pos.getRadians() - mTargetArmState.pos.getRadians());
//         setpointAngle = mTargetState.pos.getRadians();
//         // setpointAngle = mTargetState.pos.getRadians() - mRushinatorPivot.getPivotAngle().getRadians();
//         SmartDashboard.putNumber("Current Arm Pivot (Radians)", setpointAngle);
//         SmartDashboard.putNumber("Actual Setpoint (Rotations)", Units.radiansToRotations(setpointAngle));
//         // SmartDashboard.putNumber("Setpoint Angle [Degrees]", Units.radiansToDegrees(setpointAngle));
//         mPPIDController.setGoal(setpointAngle);
//     }

//     @Override
//     public void execute() {
//         var pidVoltage = mPPIDController.calculate(mRushinatorWrist.getCurrentRelativePos().getRadians());
//         // var pidVoltage = 0.0;
//         var ffVoltage = mFFController.calculate(mPPIDController.getSetpoint().velocity);

//         var error = setpointAngle - mRushinatorWrist.getCurrentRelativePos().getRadians();

//         if (error <= 1.0) {
//             ffVoltage = 0.0;
//         }
//         // var ffVoltage = 0.0;
//         var totalVoltage = ffVoltage + pidVoltage;
//         mRushinatorWrist.setVoltage(totalVoltage);

//         SmartDashboard.putNumber("Error (radians)", error);
//         SmartDashboard.putBoolean("At Wrist Target", mPPIDController.atGoal());
//         SmartDashboard.putNumber("PID Error", mPPIDController.getPositionError());
//         SmartDashboard.putNumber("PID Setpoint", mPPIDController.getSetpoint().position);
//         SmartDashboard.putNumber("PID Output Voltage", pidVoltage);
//         SmartDashboard.putNumber("FF Output Voltage", ffVoltage);
//         SmartDashboard.putNumber("Total Voltage", totalVoltage);
//     }

//     @Override
//     public boolean isFinished() {
//         return false;
//     }

//     @Override
//     public void end(boolean interrupted) {
//         mRushinatorWrist.setVoltage(0);
//     }

// public class SetWristState extends Command {
//     private final RushinatorWrist mRushinatorWrist;
//     private final TalonFX mWristMotor; // Use TalonFX motor controller
//     private final RushinatorWrist.State mTargetState;
//     private double setpointAngle;

//     public SetWristState(RushinatorWrist.State targetWristState) {
//         mRushinatorWrist = RushinatorWrist.getInstance();
//         mWristMotor = mRushinatorWrist.mWristTalon; // Get the TalonFX motor instance
//         mTargetState = targetWristState;

//         // Add requirements for the wrist subsystem
//         addRequirements(mRushinatorWrist);
//     }

//     @Override
//     public void initialize() {
//         // Calculate the setpoint angle relative to the current wrist position
//         // setpointAngle = mTargetState.pos.getRadians() - mRushinatorWrist.getCurrentRelativePos().getRadians();
//         setpointAngle = mTargetState.pos.getRadians();

//         // Log the calculated setpoint for debugging
//         SmartDashboard.putNumber("Setpoint Angle (Radians)", setpointAngle);
//         SmartDashboard.putNumber("Setpoint Angle (Degrees)", Units.radiansToDegrees(setpointAngle));

//         // Configure the TalonFX PID and Motion Magic settings using Slot0
//         TalonFXConfiguration config = new TalonFXConfiguration();

//         // Set the PID constants (P, I, D)
//         config.Slot0.kP = Settings.kP;
//         config.Slot0.kI = Settings.kI;
//         config.Slot0.kD = Settings.kD;

//         // Set Feedforward constants (if any) for the motor
//         config.Slot0.kG = Settings.kG;
//         config.Slot0.kS = Settings.kS;
//         config.Slot0.kV = Settings.kV;
//         config.Slot0.kA = Settings.kA;

//         // Set Motion Magic specific settings like velocity and acceleration in degrees
//         config.MotionMagic.MotionMagicCruiseVelocity = Units.radiansToDegrees(Settings.kMaxVelocity.getRadians());
//         config.MotionMagic.MotionMagicAcceleration = Units.radiansToDegrees(Settings.kMaxAcceleration.getRadians());

//         // Apply the configuration to the TalonFX motor
//         mWristMotor.getConfigurator().apply(config);
        
//         final MotionMagicVoltage mRequest = new MotionMagicVoltage(0);
//         // final MotionMagicDutyCycle mRequest = new MotionMagicDutyCycle(0.0);
//         // Set the target position (Motion Magic mode)
//         mWristMotor.setControl(mRequest.withPosition(setpointAngle));

//     }

//     @Override
//     public void execute() {
//         // Log the current position, target position, and error
//         double currentPosition = Units.radiansToDegrees(mRushinatorWrist.getCurrentRelativePos().getRadians());
//         double targetPosition = Units.radiansToDegrees(setpointAngle);
//         double error = targetPosition - currentPosition;

//         // Log relevant information to SmartDashboard
//         SmartDashboard.putNumber("Current Wrist Position (Rotations)", Units.degreesToRotations(currentPosition));
//         SmartDashboard.putNumber("Target Wrist Position (Rotations)", Units.degreesToRotations(targetPosition));
//         SmartDashboard.putNumber("Position Error (Rotations)", Units.degreesToRotations(error));
//         SmartDashboard.putNumber("Wrist Motor Output Voltage", RushinatorWrist.getInstance().getMotorOutputVoltage());

//         // Check if the motor has reached the target using the Motion Magic `getClosedLoopError`
//         if (Math.abs(error) < 2.0) { // You can adjust the tolerance here as needed
//             SmartDashboard.putBoolean("At Target", true);
//         } else {
//             SmartDashboard.putBoolean("At Target", false);
//         }
//     }

//     @Override
//     public boolean isFinished() {
//         // The command will finish when the wrist is at the target position
//         return false;
//         // return Math.abs(Units.radiansToDegrees(mRushinatorWrist.getCurrentRelativePos().getRadians()) - Units.radiansToDegrees(setpointAngle)) < 2.0; // Adjust tolerance
//     }

//     @Override
//     public void end(boolean interrupted) {
//         // Stop the wrist motor when the command ends
//         mWristMotor.setVoltage(0.0);
//     }
}
