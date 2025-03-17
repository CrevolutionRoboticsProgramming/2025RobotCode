package frc.robot.rushinator.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.rushinator.RushinatorWrist;

public class HoldWristPosition extends Command {
    private double baselineCancoderRadians;

    public HoldWristPosition() {
        addRequirements(RushinatorWrist.getInstance());
    }

    @Override
    public void initialize() {
        // On initialization, hold the last commanded state if available;
        // otherwise, use the current sensor reading to set the goal.
        baselineCancoderRadians = RushinatorWrist.getInstance().getCurrentAngle();
        if (RushinatorWrist.getInstance().getCurrentWristState() != null) {
            RushinatorWrist.getInstance().setTargetState(RushinatorWrist.getInstance().getCurrentWristState());
        } else {
            double currentRad = RushinatorWrist.getInstance().getCurrentAngle();
            RushinatorWrist.getInstance().setTargetPosition(new Rotation2d(currentRad));
        }
    }

    @Override
    public void execute() {
        double desiredWristRadians = RushinatorWrist.getInstance().getCurrentWristState().pos.getRadians();
        double delta = RushinatorWrist.getInstance().getCurrentAngle() - baselineCancoderRadians;
        double compensatedGoal = desiredWristRadians - delta;
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