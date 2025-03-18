package frc.robot.rushinator.commands;

import java.lang.invoke.VolatileCallSite;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.rushinator.RushinatorPivot;
import frc.robot.rushinator.RushinatorWrist;
import frc.robot.rushinator.RushinatorWrist.Settings;
import frc.robot.rushinator.RushinatorWrist.State;

public class SetWristState extends Command{
    State mCurrentState = RushinatorWrist.getInstance().getCurrentWristState();
    State mTargetState;
    private final ProfiledPIDController mPPIDController;
    private final SimpleMotorFeedforward mFFController;
    private final RushinatorWrist mRushWrist;

    public SetWristState(State targetWristState) {
        mTargetState = targetWristState;

         mPPIDController = new ProfiledPIDController(Settings.kP, Settings.kI, Settings.kD, new TrapezoidProfile.Constraints(
                Settings.kMaxVelocity.getRadians(),
                Settings.kMaxAcceleration.getRadians()
        ));
        mPPIDController.setTolerance(1); //degrees of tolerance

        mFFController = new SimpleMotorFeedforward(Settings.kS, Settings.kV, Settings.kA);

        mRushWrist = RushinatorWrist.getInstance();
        addRequirements(RushinatorWrist.getInstance());
    }

    @Override
    public void initialize() {
        // mPPIDController.setGoal(mTargetState.pos.getRadians());
        SmartDashboard.putNumber("SetPoint", mTargetState.pos.getRotations());
        Rotation2d offSet = Rotation2d.fromRotations(-89.0 * RushinatorPivot.getInstance().getArmPosition().getRotations() + 13.3);
        mPPIDController.setGoal(mTargetState.pos.getRadians() + offSet.getRadians());
    }

    @Override
    public void execute() {

        var voltage = mPPIDController.calculate(mRushWrist.getCurrentPos().getRadians());
        voltage += mFFController.calculate(mPPIDController.getSetpoint().velocity);
        // voltage += mFFController.calculate(mRushWrist.getCurrentPos().getRadians(), mPPIDController.getSetpoint().velocity);
        mRushWrist.setVoltage(voltage);

        SmartDashboard.putNumber("PID Output", mPPIDController.calculate(mRushWrist.getCurrentPos().getRadians()));
        SmartDashboard.putNumber("FF Output", mFFController.calculate(mRushWrist.getCurrentPos().getRadians(), mPPIDController.getSetpoint().velocity));
        SmartDashboard.putNumber("Output Voltage", voltage);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        // mRushWrist.setVoltage(0);
    }
}
