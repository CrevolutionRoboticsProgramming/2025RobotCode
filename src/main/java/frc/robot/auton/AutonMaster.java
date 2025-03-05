package frc.robot.auton;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;


import edu.wpi.first.wpilibj.DriverStation;
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
        configureNamedCommands();

        // Configuring AutoBuilder
        // AutoBuilder.configure(
        //     drivetrain::getPose,
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
       
        configurePathPlannerLogging();
    }

    public void configureNamedCommands() {
       
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