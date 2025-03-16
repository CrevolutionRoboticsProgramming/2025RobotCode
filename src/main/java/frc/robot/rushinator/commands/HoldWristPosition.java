package frc.robot.rushinator.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.rushinator.RushinatorWrist;

public class HoldWristPosition extends Command {
    public HoldWristPosition() {
        addRequirements(RushinatorWrist.getInstance());
    }

    @Override
    public void initialize() {
        // On initialization, hold the last commanded state if available;
        // otherwise, use the current sensor reading to set the goal.
        if (RushinatorWrist.getInstance().getCurrentWristState() != null) {
            RushinatorWrist.getInstance().setTargetState(RushinatorWrist.getInstance().getCurrentWristState());
        } else {
            double currentRad = RushinatorWrist.getInstance().getCurrentAngle();
            RushinatorWrist.getInstance().setTargetPosition(new Rotation2d(currentRad));
        }
    }

    @Override
    public void execute() {
        double compensation = 0.0; // TODO: Replace with actual arm sensor reading/calculation.

        double desiredWristRadians = RushinatorWrist.getInstance().getCurrentWristState().pos.getRadians();
        double compensatedGoal = desiredWristRadians - compensation;
        RushinatorWrist.getInstance().setTargetPosition(new Rotation2d(compensatedGoal));
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }


}