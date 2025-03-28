// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.vision.commands;

import java.util.List;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.drivetrain.CommandSwerveDrivetrain;
import frc.robot.vision.VisionConfig.GeometryUtils;
import frc.robot.vision.VisionConfig.PathPlannerConstants;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

/** This automatically drives to a pose without using A* to generate a trajectory, 
  * useful for when we know there are no obstructions on the field between the robot 
  * and the desired pose */
public class DirectDriveToPoseCommand extends Command {

    private Command followPathCommand;
    private final Pose2d targetPose;
    private final CommandSwerveDrivetrain m_drive;

    public DirectDriveToPoseCommand(CommandSwerveDrivetrain drive, Pose2d targetPose) {
        this.m_drive = drive;
        this.targetPose = targetPose;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        Pose2d currentPose = m_drive.getPose();
        Pose2d startingWaypoint = new Pose2d(currentPose.getTranslation(), GeometryUtils.angleToPose(currentPose, targetPose));
        
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
            startingWaypoint,
            targetPose
        );

        // Prevent PathPlanner from treating the start pose as identical to the end pose when they are too close to each other
        if (targetPose.getTranslation().getDistance(startingWaypoint.getTranslation()) > PathPlannerConstants.pathGenerationToleranceMeters){
            PathPlannerPath path = new PathPlannerPath(
                waypoints, 
                PathPlannerConstants.pathConstraints, 
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
        return () -> targetPose.getTranslation().getDistance(m_drive.getPose().getTranslation()) < PathPlannerConstants.LEDpathToleranceMeters;
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