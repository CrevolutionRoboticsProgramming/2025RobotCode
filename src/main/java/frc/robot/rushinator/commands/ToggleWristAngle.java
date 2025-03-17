package frc.robot.rushinator.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.rushinator.RushinatorWrist.State;
import frc.robot.rushinator.RushinatorWrist;

public class ToggleWristAngle extends Command {
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