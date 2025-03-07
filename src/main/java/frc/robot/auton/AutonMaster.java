package frc.robot.auton;

import static edu.wpi.first.units.Units.Rotation;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.driver.DriverXbox;
import frc.robot.drivetrain.CommandSwerveDrivetrain;

/* MASTER AUTON CLASS */
public class AutonMaster {
    // private static Field2d mGameField;
    private static SendableChooser<Command> autonChooser = new SendableChooser<>();
    private static CommandSwerveDrivetrain drivetrain;

    private static AutonMaster mInstance;

    
    public AutonMaster() {
        drivetrain = CommandSwerveDrivetrain.getInstance();
        try {
            var config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                drivetrain::getPose,   // Supplier of current robot pose
                drivetrain::resetPose,         // Consumer for seeding pose against auto
                drivetrain::getRobotRelvativeSpeeds, // Supplier of current robot speeds
                // Consumer of ChassisSpeeds and feedforwards to drive the robot
                (speeds, feedforwards) -> drivetrain.setControl(
                    drivetrain.m_pathApplyRobotSpeeds.withSpeeds(speeds)
                        .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                        .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
                ),
                new PPHolonomicDriveController(
                    // PID constants for translation
                    new PIDConstants(10, 0.0, 0.0),
                    // PID constants for rotation
                    new PIDConstants(6.5, 0, 0.05)
                ),
                config,
                // Assume the path needs to be flipped for Red vs Blue, this is normally the case
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                drivetrain // Subsystem for requirements
            );
        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
        }

        configureNamedCommands();
        configurePathPlannerLogging();
        
        //Add Auton Options Here
        autonChooser.addOption("TestPath", AutoBuilder.buildAuto("TestAuto"));
    }
    

    public static AutonMaster getInstance() {
        if (mInstance == null) {
            mInstance = new AutonMaster();
        }
        return mInstance;
    }

    public void configureNamedCommands() {
       
    }

    public Command getTestPathFindingCommand() {
        PathConstraints pathFindConstraints = new PathConstraints(3.00, 3.00, Units.degreesToRadians(540), Units.degreesToRadians(720));
        Translation2d pathFindGoalPoseTranslation = new Translation2d(1.202, 2.169);
        Rotation2d pathFindGoalPoseRotation = Rotation2d.fromDegrees(0);
        Pose2d pathFindGoalPose = new Pose2d(pathFindGoalPoseTranslation, pathFindGoalPoseRotation);
        return AutoBuilder.pathfindToPose(pathFindGoalPose, pathFindConstraints);
    }

    public SendableChooser<Command> getAutonSelector() {
        return autonChooser;
    }

    private void configurePathPlannerLogging() {
        // mGameField = new Field2d();
        // SmartDashboard.putData("Field", mGameField);

        // PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
        //     mGameField.setRobotPose(pose);
        // });

        // PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
        //     mGameField.getObject("target pose").setPose(pose);
        // });

        // PathPlannerLogging.setLogActivePathCallback((poses) -> {
        //     mGameField.getObject("path").setPoses(poses);
        // });
    }
}