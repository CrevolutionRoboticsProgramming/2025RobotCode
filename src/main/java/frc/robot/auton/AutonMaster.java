package frc.robot.auton;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;


import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.drivetrain.CommandSwerveDrivetrain;

/* MASTER AUTON CLASS */
public class AutonMaster {
    private static Field2d mGameField;
    private static SendableChooser<Command> autonChooser = new SendableChooser<>();
    private static CommandSwerveDrivetrain drivetrain;

    // private final Drivetrain drivetrain;

    public AutonMaster() {
        drivetrain = CommandSwerveDrivetrain.getInstance();
        /* Define Named Commands Here */
        // configureNamedCommands();

        // Configuring AutoBuilder
        // AutoBuilder.configure(
        //     drivetrain::getState().Pose,
        //     drivetrain::resetPose,
        //     drivetrain::getRobotRelativeSpeeds,
        //     drivetrain::driveRobotRelative,
        //     new PPHolonomicDriveController(
        //         AutonConfig.TRANSLATION_PID,
        //         AutonConfig.ROTATION_PID
        //     ),
        //     new RobotConfig(
        //         DriveConstants.robotKG,
        //         DriveConstants.MOI,
        //         DriveConstants.modConfig,
        //         DriveConstants.modsOffSets.get()
        //     ),
        //     () -> {
        //         var alliance = DriverStation.getAlliance();
        //         if (alliance.isPresent()) {
        //             return alliance.get() == DriverStation.Alliance.Red;
        //         }
        //         return false;
        //     },
        //     drivetrain
        // );

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
                    new PIDConstants(10, 0, 0),
                    // PID constants for rotation
                    new PIDConstants(7, 0, 0)
                ),
                config,
                // Assume the path needs to be flipped for Red vs Blue, this is normally the case
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                drivetrain // Subsystem for requirements
            );
        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
        }
        // configureAutoBuilder(drivetrain);
        configureNamedCommands();
        configurePathPlannerLogging();

        autonChooser.addOption("TestPath", AutoBuilder.buildAuto("TestAuto"));
    }

    public void configureNamedCommands() {
       
    }


    private void configureAutoBuilder(CommandSwerveDrivetrain drivetrain) {
        
    }


    public SendableChooser<Command> getAutonSelector() {
        return autonChooser;
    }

    private void configurePathPlannerLogging() {
        mGameField = new Field2d();
        SmartDashboard.putData("Field", mGameField);

        PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
            mGameField.setRobotPose(pose);
        });

        PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
            mGameField.getObject("target pose").setPose(pose);
        });

        PathPlannerLogging.setLogActivePathCallback((poses) -> {
            mGameField.getObject("path").setPoses(poses);
        });
    }
}