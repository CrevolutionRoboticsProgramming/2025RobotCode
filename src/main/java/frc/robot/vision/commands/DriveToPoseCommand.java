package frc.robot.vision.commands;

import java.util.List;
import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.crevolib.math.Conversions;
import frc.robot.drivetrain.CommandSwerveDrivetrain;
import frc.robot.vision.PoseEstimatorSubsystem;

public class DriveToPoseCommand extends Command{
    private Command followPathCommand;
    private final Pose2d targetPose;
    CommandSwerveDrivetrain mDrivetrain;
    public final PathConstraints pathConstraints = new PathConstraints(2, 2, Units.degreesToRadians(360), Units.degreesToRadians(360));
    PoseEstimatorSubsystem mPoseEstimatorSubsystem;

    public DriveToPoseCommand(Pose2d targetPose) {
        this.mDrivetrain = CommandSwerveDrivetrain.getInstance();
        this.mPoseEstimatorSubsystem = PoseEstimatorSubsystem.getInstance();
        this.targetPose = targetPose;
        addRequirements(mDrivetrain, mPoseEstimatorSubsystem);
    }

    @Override
    public void initialize() {
        Pose2d currentPose = mPoseEstimatorSubsystem.getCurrentPose();
        Pose2d startingWaypoint = new Pose2d(currentPose.getTranslation(), Conversions.angleToPose(currentPose, targetPose));
        
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
            startingWaypoint,
            targetPose
        );

        // Prevent PathPlanner from treating the start pose as identical to the end pose when they are too close to each other
        if (targetPose.getTranslation().getDistance(startingWaypoint.getTranslation()) > 0.011){
            PathPlannerPath path = new PathPlannerPath(
                waypoints, 
                this.pathConstraints, 
                //new IdealStartingState(m_drive.getVelocityMPS(), m_drive.getHeading()), 
                null,
                new GoalEndState(0, targetPose.getRotation())
            );
            path.preventFlipping = true;
            followPathCommand = AutoBuilder.followPath(path);
        } else {
            followPathCommand = Commands.none();
        }
        followPathCommand.schedule();
    }

    public BooleanSupplier atGoal() {
        return () -> targetPose.getTranslation().getDistance(mPoseEstimatorSubsystem.getCurrentPose().getTranslation()) < 0.03;
    }

    @Override
    public void execute(){
        followPathCommand.execute();
    }
    
    @Override
    public boolean isFinished() {
        return followPathCommand.isFinished();
    }
    
    @Override
    public void end(boolean interrupted) {
        followPathCommand.end(interrupted);
    }
}
