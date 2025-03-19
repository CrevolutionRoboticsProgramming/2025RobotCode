package frc.robot.rushinator.commands;

import java.lang.invoke.VolatileCallSite;

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
    private final ProfiledPIDController mPPIDController;
    private final SimpleMotorFeedforward mFFController;
    private final RushinatorWrist.State mTargetState;
    private double setpointAngle;

    public SetWristState(RushinatorWrist.State targetWristState) {
        mRushinatorWrist = RushinatorWrist.getInstance();

        mPPIDController = new ProfiledPIDController(RushinatorWrist.Settings.kP, RushinatorWrist.Settings.kI, RushinatorWrist.Settings.kD, new TrapezoidProfile.Constraints(
                            RushinatorWrist.Settings.kMaxVelocity.getRadians(),
                            RushinatorWrist.Settings.kMaxAcceleration.getRadians()
                    ));
        mPPIDController.setTolerance(1); //degrees of tolerance

        mTargetState = targetWristState;

        mFFController = new SimpleMotorFeedforward(Settings.kS, Settings.kV, Settings.kA);

        mRushinatorPivot = RushinatorPivot.getInstance();

        addRequirements(mRushinatorWrist);
    }

    @Override
    public void initialize() {
        setpointAngle = mTargetState.pos.getRadians() + mRushinatorPivot.getPivotAngle().getRadians();
        SmartDashboard.putNumber("Setpoint Angle [Degrees]", Units.radiansToDegrees(setpointAngle));
        mPPIDController.setGoal(setpointAngle);
    }

    @Override
    public void execute() {
        var pidVoltage = mPPIDController.calculate(mRushinatorWrist.getWristAngle().getRadians());
        var ffVoltage = mFFController.calculate(mPPIDController.getSetpoint().velocity);
        var totalVoltage = pidVoltage + ffVoltage;
        mRushinatorWrist.setVoltage(totalVoltage);

        SmartDashboard.putNumber("PID Output Voltage", pidVoltage);
        SmartDashboard.putNumber("FF Output Voltage", ffVoltage);
        SmartDashboard.putNumber("Total Voltage", totalVoltage);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        mRushinatorWrist.setVoltage(0);
    }
}
