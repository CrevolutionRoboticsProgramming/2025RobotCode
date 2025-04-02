// package frc.robot.vision.commands;

// import java.util.List;
// import java.util.function.BooleanSupplier;

// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.config.PIDConstants;
// import com.pathplanner.lib.config.RobotConfig;
// import com.pathplanner.lib.controllers.PPHolonomicDriveController;
// import com.pathplanner.lib.path.GoalEndState;
// import com.pathplanner.lib.path.PathConstraints;
// import com.pathplanner.lib.path.PathPlannerPath;
// import com.pathplanner.lib.path.Waypoint;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.DriverStation.Alliance;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import frc.crevolib.math.Conversions;
// import frc.robot.drivetrain.CommandSwerveDrivetrain;
// import frc.robot.vision.PoseEstimatorSubsystem;

// public class DriveToPoseCommand extends Command{
//     private Command followPathCommand;
//     private final Pose2d targetPose;
//     public final PathConstraints pathConstraints = new PathConstraints(
//         2, 
//         2, 
//         Units.degreesToRadians(360), 
//         Units.degreesToRadians(360));
//     PoseEstimatorSubsystem mPoseEstimatorSubsystem;
//     private Timer mTimer;
//     public DriveToPoseCommand(Pose2d targetPose) {
//         // try {
//         //     var config = RobotConfig.fromGUISettings();
//         //     AutoBuilder.configure(
//         //         CommandSwerveDrivetrain.getInstance()::getPose,   // Supplier of current robot pose
//         //         CommandSwerveDrivetrain.getInstance()::resetPose,         // Consumer for seeding pose against auto
//         //         CommandSwerveDrivetrain.getInstance()::getRobotRelvativeSpeeds, // Supplier of current robot speeds
//         //         // Consumer of ChassisSpeeds and feedforwards to drive the robot
//         //         (speeds, feedforwards) -> CommandSwerveDrivetrain.getInstance().setControl(
//         //             CommandSwerveDrivetrain.getInstance().m_pathApplyRobotSpeeds.withSpeeds(speeds)
//         //                 .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
//         //                 .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
//         //         ),
//         //         new PPHolonomicDriveController(
//         //             // PID constants for translation
//         //             new PIDConstants(.025, 0.0, 0.0),
//         //             // PID constants for rotation
//         //             new PIDConstants(.02, 0, 0.05)
//         //         ),
//         //         config,
//         //         // Assume the path needs to be flipped for Red vs Blue, this is normally the case
//         //         () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
//         //         CommandSwerveDrivetrain.getInstance() // Subsystem for requirements
//         //     );
//         // } catch (Exception ex) {
//         //     DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
//         // }

//         mTimer = new Timer();

//         this.mPoseEstimatorSubsystem = PoseEstimatorSubsystem.getInstance();
//         this.targetPose = targetPose;
//         addRequirements(mPoseEstimatorSubsystem);
//     }

//     @Override
//     public void initialize() {
//         Pose2d currentPose = mPoseEstimatorSubsystem.getCurrentPose();
//         Pose2d startingWaypoint = new Pose2d(currentPose.getTranslation(), Conversions.angleToPose(currentPose, targetPose));
        
//         List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
//             startingWaypoint,
//             targetPose
//         );

//         // Prevent PathPlanner from treating the start pose as identical to the end pose when they are too close to each other
//         if (targetPose.getTranslation().getDistance(startingWaypoint.getTranslation()) > 0.011){
//             PathPlannerPath path = new PathPlannerPath(
//                 waypoints, 
//                 this.pathConstraints, 
//                 //new IdealStartingState(m_drive.getVelocityMPS(), m_drive.getHeading()), 
//                 null,
//                 new GoalEndState(0, targetPose.getRotation())
//             );
//             path.preventFlipping = true;
//             followPathCommand = AutoBuilder.followPath(path);
//         } else {
//             followPathCommand = Commands.none();
//         }
//         followPathCommand.schedule();
//         SmartDashboard.putString("current pose lineup", currentPose.toString());
//         SmartDashboard.putString("target pose lineup", targetPose.toString());
//         SmartDashboard.putString("starting waypoint lineup", startingWaypoint.toString());
//     }

//     public BooleanSupplier atGoal() {
//         return () -> targetPose.getTranslation().getDistance(mPoseEstimatorSubsystem.getCurrentPose().getTranslation()) < 0.03;
//     }

//     @Override
//     public void execute(){
//         followPathCommand.execute();
//         mTimer.start();
//     }
    
//     @Override
//     public boolean isFinished() {
//         return followPathCommand.isFinished() || mTimer.get() > 2;
//     }
    
//     @Override
//     public void end(boolean interrupted) {
//         followPathCommand.end(interrupted);
//     }
// }