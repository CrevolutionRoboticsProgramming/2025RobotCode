package frc.robot.rushinator.commands;

import java.net.CookieManager;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.rushinator.RushinatorPivot;
import frc.robot.rushinator.RushinatorWrist;
import frc.robot.rushinator.RushinatorWrist.State;

public class ToggleWristState extends Command{
    RushinatorWrist mRushinatorWrist;
    Rotation2d targetAngle;
    RushinatorWrist.State kTargetState;

    public ToggleWristState() {
        mRushinatorWrist = RushinatorWrist.getInstance();  
        // targetAngle = targetState.pos;
        addRequirements(mRushinatorWrist);
    }

    @Override
    public void initialize() {
        System.out.println("Last State in Toggle Wrist" + RushinatorWrist.kLastState.name());
        if(RushinatorWrist.kLastState == State.kScoreLeftWrist) {
            System.out.println("First If");
            mRushinatorWrist.setTargetState(RushinatorWrist.State.kScoreRightWrist);
        }
        else if(RushinatorWrist.kLastState == State.kScoreRightWrist) {
            System.out.println("2nd If");
            mRushinatorWrist.setTargetState(RushinatorWrist.State.kScoreLeftWrist);
        }
        else if(RushinatorWrist.kLastState == State.kTravelLeft) {
            System.out.println("3rd If");
            mRushinatorWrist.setTargetState(RushinatorWrist.State.kTravelRight);
        }
        else if(RushinatorWrist.kLastState == State.kTravelRight) {
            System.out.println("Fourth If");
            mRushinatorWrist.setTargetState(RushinatorWrist.State.kTravelLeft);
        }
        else {
            System.out.println("Else");
            mRushinatorWrist.setTargetState(RushinatorWrist.kLastState);
        }
    }

    @Override
    public void end(boolean interrupted) {

    }
}
