package frc.robot.driver;


import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.crevolib.util.ExpCurve;
import frc.crevolib.util.XboxGamepad;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.algaeflywheel.AlgaeRoller;
import frc.robot.algaepivot.AlgaeSubsystem;
import frc.robot.algaepivot.commands.AlgaePivotCommands;
import frc.robot.algaepivot.commands.SetAngleAlgaePivot;
import frc.robot.auton.AutonMaster;
import frc.robot.climber.Climber;
import frc.robot.climber.commands.SetClimberAngle;
import frc.robot.commands.RobotCommands;
import frc.robot.drivetrain.CommandSwerveDrivetrain;
import frc.robot.elevator.ElevatorSubsystem;
import frc.robot.elevator.commands.ElevatorCommands;
import frc.robot.elevator.commands.SetElevatorState;
import frc.robot.indexer.commands.IndexerCommands;
import frc.robot.operator.OperatorXbox;
import frc.robot.rushinator.RushinatorPivot;
import frc.robot.rushinator.RushinatorWrist;
import frc.robot.rushinator.commands.SetArmState;
import frc.robot.rushinator.commands.SetWristState;
import frc.robot.rushinator.commands.ToggleWristState;
import frc.robot.rushinator.commands.SetRollersVoltage;
import frc.robot.vision.LineupMaster;
import frc.robot.vision.PoseEstimatorSubsystem;
import frc.robot.vision.VisionConfig;
import frc.robot.vision.VisionConfig.HPStation;
import frc.robot.vision.commands.AutoAlign;
import frc.robot.vision.commands.AutoAlignHP;

public class DriverXbox extends XboxGamepad {
    private static class Settings {
        static final int port = 0;
        static final String name = "driver";

        static final double kTranslationExpVal = 4.0;
        static final double kRotationExpVal = 1.0;
        static final double kDeadzone = 0.01;
    }

    private static DriverXbox mInstance;
    public static ExpCurve translationStickCurve;
    private static ExpCurve rotationStickCurve;
    public boolean autoAim;
    private double reqAngularVel;
    private static AlgaeSubsystem mAlgaeSubsystem;

    private static LineupMaster mLineupMaster;
    

    private DriverXbox() {
        super(DriverXbox.Settings.name, DriverXbox.Settings.port);

        mLineupMaster = new LineupMaster();

        translationStickCurve = new ExpCurve(DriverXbox.Settings.kTranslationExpVal, 0, 1, DriverXbox.Settings.kDeadzone);
        rotationStickCurve = new ExpCurve(DriverXbox.Settings.kRotationExpVal, 0, 1, DriverXbox.Settings.kDeadzone);
    }

    public static DriverXbox getInstance() {
        if (mInstance == null) {
            mInstance = new DriverXbox();
        }
        return mInstance;
    }
    

    @Override
    public void setupTeleopButtons() {
        /*COMP BINDINGS */
        // Spit Coral
        controller.x().whileTrue(new SetRollersVoltage(-2.0));
        
        //Score Coral
        // controller.y().whileTrue(RobotCommands.scoreCoral());
        controller.y().and(() -> OperatorXbox.getInstance().controller.a().getAsBoolean()).whileTrue(new ConditionalCommand(
            RobotCommands.coralPrimeShoot(RushinatorPivot.State.kScoreL1, RushinatorWrist.State.kScoreL1Mid), 
            RobotCommands.coralPrimeShoot(RushinatorPivot.State.kScoreL1, RushinatorWrist.State.kScoreL1Mid), 
            () -> RushinatorWrist.kLastState == RushinatorWrist.State.kTravelRight || 
            RushinatorWrist.kLastState == RushinatorWrist.State.kScoreRightWrist || 
            RushinatorWrist.kLastState == RushinatorWrist.State.kScoreL1Mid || 
            RushinatorWrist.kLastState == RushinatorWrist.State.kGroundMid || 
            RushinatorWrist.kLastState == RushinatorWrist.State.kScoreL4RightWrist || 
            RushinatorWrist.kLastState == RushinatorWrist.State.kScoreL3RightWrist || 
            RushinatorWrist.kLastState == RushinatorWrist.State.kScoreL2RightWrist || 
            RushinatorWrist.kLastState == RushinatorWrist.State.kTravelL4Right ||
            RushinatorWrist.kLastState == RushinatorWrist.State.kHPMid)
        );
        controller.y().and(() -> OperatorXbox.getInstance().controller.a().getAsBoolean()).whileTrue(
            new SetRollersVoltage(-1.2)
        );

        controller.y().and(() -> OperatorXbox.getInstance().controller.x().getAsBoolean()).whileTrue(new ConditionalCommand(
            RobotCommands.coralPrimeShoot(RushinatorPivot.State.kScoreL2, RushinatorWrist.State.kScoreL2RightWrist), 
            RobotCommands.coralPrimeShoot(RushinatorPivot.State.kScoreL2, RushinatorWrist.State.kScoreL2LeftWrist), 
            () -> RushinatorWrist.kLastState == RushinatorWrist.State.kTravelRight || 
            RushinatorWrist.kLastState == RushinatorWrist.State.kScoreRightWrist || 
            RushinatorWrist.kLastState == RushinatorWrist.State.kScoreL2RightWrist || 
            RushinatorWrist.kLastState == RushinatorWrist.State.kScoreL4RightWrist || 
            RushinatorWrist.kLastState == RushinatorWrist.State.kScoreL3RightWrist || 
            RushinatorWrist.kLastState == RushinatorWrist.State.kGroundMid || 
            RushinatorWrist.kLastState == RushinatorWrist.State.kTravelL4Right ||
            RushinatorWrist.kLastState == RushinatorWrist.State.kScoreL1Mid ||
            RushinatorWrist.kLastState == RushinatorWrist.State.kHPMid)
        );
        controller.y().and(() -> OperatorXbox.getInstance().controller.x().getAsBoolean()).whileTrue(
            new SetElevatorState(ElevatorSubsystem.State.kCoralScoreL2)
        );
        controller.y().and(() -> OperatorXbox.getInstance().controller.x().getAsBoolean()).whileTrue(
            new SetRollersVoltage(0.0)
        );
        controller.y().and(() -> OperatorXbox.getInstance().controller.x().getAsBoolean()).whileFalse(
            new SetElevatorState(ElevatorSubsystem.State.kCoralL2)
        );

        controller.y().and(() -> OperatorXbox.getInstance().controller.b().getAsBoolean()).whileTrue(new ConditionalCommand(
            RobotCommands.coralPrimeShoot(RushinatorPivot.State.kScoreL3, RushinatorWrist.State.kScoreL3RightWrist), 
            RobotCommands.coralPrimeShoot(RushinatorPivot.State.kScoreL3, RushinatorWrist.State.kScoreL3LeftWrist), 
            () -> RushinatorWrist.kLastState == RushinatorWrist.State.kTravelRight || 
            RushinatorWrist.kLastState == RushinatorWrist.State.kScoreRightWrist || 
            RushinatorWrist.kLastState == RushinatorWrist.State.kScoreL3RightWrist || 
            RushinatorWrist.kLastState == RushinatorWrist.State.kGroundMid || 
            RushinatorWrist.kLastState == RushinatorWrist.State.kScoreL4RightWrist || 
            RushinatorWrist.kLastState == RushinatorWrist.State.kScoreL2RightWrist || 
            RushinatorWrist.kLastState == RushinatorWrist.State.kTravelL4Right ||
            RushinatorWrist.kLastState == RushinatorWrist.State.kScoreL1Mid ||
            RushinatorWrist.kLastState == RushinatorWrist.State.kHPMid)
        );
        controller.y().and(() -> OperatorXbox.getInstance().controller.b().getAsBoolean()).whileTrue(
            new SetElevatorState(ElevatorSubsystem.State.kCoralScoreL3)
        );
        controller.y().and(() -> OperatorXbox.getInstance().controller.b().getAsBoolean()).whileTrue(
            new SetRollersVoltage(0.0)
        );
        controller.y().and(() -> OperatorXbox.getInstance().controller.b().getAsBoolean()).whileFalse(
            new SetElevatorState(ElevatorSubsystem.State.kCoralL3)
        );
        
        controller.y().and(() -> OperatorXbox.getInstance().controller.y().getAsBoolean()).whileTrue(new ConditionalCommand(
            RobotCommands.coralPrimeShoot(RushinatorPivot.State.kScoreL4, RushinatorWrist.State.kScoreL4RightWrist), 
            RobotCommands.coralPrimeShoot(RushinatorPivot.State.kScoreL4, RushinatorWrist.State.kScoreL4LeftWrist), 
            () -> RushinatorWrist.kLastState == RushinatorWrist.State.kTravelRight || 
            RushinatorWrist.kLastState == RushinatorWrist.State.kScoreRightWrist || 
            RushinatorWrist.kLastState == RushinatorWrist.State.kScoreL4RightWrist || 
            RushinatorWrist.kLastState == RushinatorWrist.State.kScoreL3RightWrist || 
            RushinatorWrist.kLastState == RushinatorWrist.State.kScoreL2RightWrist || 
            RushinatorWrist.kLastState == RushinatorWrist.State.kTravelL4Right ||
            RushinatorWrist.kLastState == RushinatorWrist.State.kGroundMid || 
            RushinatorWrist.kLastState == RushinatorWrist.State.kScoreL1Mid ||
            RushinatorWrist.kLastState == RushinatorWrist.State.kHPMid)
        );
        controller.y().and(() -> OperatorXbox.getInstance().controller.y().getAsBoolean()).whileTrue(
            new SetElevatorState(ElevatorSubsystem.State.kCoralScoreL4)
        );
        controller.y().and(() -> OperatorXbox.getInstance().controller.y().getAsBoolean()).whileTrue(
            new SetRollersVoltage(0.0)
        );
        

        // Score L4 Auto Align
        // controller.y().and(() -> OperatorXbox.getInstance().controller.x().and(leftTriggerOnly()).getAsBoolean()).whileTrue(new ConditionalCommand(
        //     RobotCommands.coralPrimeShoot(RushinatorPivot.State.kScoreL4, RushinatorWrist.State.kScoreL4RightWrist), 
        //     RobotCommands.coralPrimeShoot(RushinatorPivot.State.kScoreL4, RushinatorWrist.State.kScoreL4LeftWrist), 
        //     () -> RushinatorWrist.kLastState == RushinatorWrist.State.kTravelRight || 
        //     RushinatorWrist.kLastState == RushinatorWrist.State.kScoreRightWrist || 
        //     RushinatorWrist.kLastState == RushinatorWrist.State.kScoreL4RightWrist || 
        //     RushinatorWrist.kLastState == RushinatorWrist.State.kScoreL3RightWrist ||  
        //     RushinatorWrist.kLastState == RushinatorWrist.State.kScoreL2RightWrist || 
        //     RushinatorWrist.kLastState == RushinatorWrist.State.kTravelL4Right ||
        //     RushinatorWrist.kLastState == RushinatorWrist.State.kGroundMid || 
        //     RushinatorWrist.kLastState == RushinatorWrist.State.kScoreL1Mid ||
        //     RushinatorWrist.kLastState == RushinatorWrist.State.kHPMid)
        // );
        // controller.y().and(() -> OperatorXbox.getInstance().controller.x().and(leftTriggerOnly()).getAsBoolean()).whileTrue(
        //     new SetElevatorState(ElevatorSubsystem.State.kCoralScoreL4)
        // );
        // controller.y().and(() -> OperatorXbox.getInstance().controller.x().and(leftTriggerOnly()).getAsBoolean()).whileTrue(
        //     new SetRollersVoltage(0.0)
        // );

        // controller.y().and(() -> OperatorXbox.getInstance().controller.y().getAsBoolean()).whileFalse(
        //     new SetElevatorState(ElevatorSubsystem.State.kCoralL4)
        // );

        // controller.y().whileTrue(new ConditionalCommand(
        //     RobotCommands.coralPrimeShoot(RushinatorPivot.State.kScore, RushinatorWrist.State.kScoreRightWrist), 
        //     RobotCommands.coralPrimeShoot(RushinatorPivot.State.kScore, RushinatorWrist.State.kScoreLeftWrist), 
        //     () -> RushinatorWrist.kLastState == RushinatorWrist.State.kTravelRight || 
        //     RushinatorWrist.kLastState == RushinatorWrist.State.kScoreRightWrist || 
        //     RushinatorWrist.kLastState == RushinatorWrist.State.kGroundMid || 
        //     RushinatorWrist.kLastState == RushinatorWrist.State.kHPMid)
        // );
        // controller.y().whileTrue(new ElevatorSubsystem.applyJog(ElevatorSubsystem.getInstance().getPosition() + 4.0));

        // Pulse Alage
        // controller.a().whileTrue(new AlgaeRoller.IntakeCommand());

         /*Algae Shoot */
        controller.b().whileTrue(new AlgaeRoller.PrimeShootCommand());
        controller.b().whileTrue(RobotCommands.algaePrime(AlgaeSubsystem.State.kScore, ElevatorSubsystem.State.kZero));
        

        // Spit Algae
        // controller.b().whileTrue(new AlgaeRoller.ShootCommand());

        // Coral Ground Intake
        controller.rightTrigger().whileTrue(RobotCommands.coralPrime(
            RushinatorPivot.State.kFloorIntake, ElevatorSubsystem.State.kZero)
        );
        controller.rightTrigger().whileTrue(
            new SetWristState(RushinatorWrist.State.kGroundMid)
        );
        controller.rightTrigger().whileTrue(new SetRollersVoltage(4.5));

        // Coral HP Intake
        controller.rightBumper().whileTrue(RobotCommands.coralPrime(
            RushinatorPivot.State.kHPIntake, ElevatorSubsystem.State.kZero)
        );
        controller.rightBumper().whileTrue(
            new SetWristState(RushinatorWrist.State.kHPMid)
        );
        controller.rightBumper().whileTrue(new SetRollersVoltage(4.5));

        // Algae Ground Intake
        controller.leftTrigger().whileTrue(new AlgaeRoller.IntakeCommand());
        controller.leftTrigger().whileTrue(new SetAngleAlgaePivot(AlgaeSubsystem.State.kFloorIntake));
        controller.leftTrigger().whileTrue(new SetElevatorState(ElevatorSubsystem.State.kAlgaeIntake));


        controller.povLeft().whileTrue(mLineupMaster.directDriveToNearestLeftBranch());
        controller.povLeft().whileTrue(new ConditionalCommand(
            RobotCommands.coralPrimeAutoScore(RushinatorPivot.State.kStowTravel, RushinatorWrist.State.kTravelRight, ElevatorSubsystem.State.kCoralL4), 
            RobotCommands.coralPrimeAutoScore(RushinatorPivot.State.kStowTravel, RushinatorWrist.State.kTravelLeft, ElevatorSubsystem.State.kCoralL4), 
            () -> RushinatorWrist.kLastState == RushinatorWrist.State.kTravelRight || 
            RushinatorWrist.kLastState == RushinatorWrist.State.kScoreRightWrist || 
            RushinatorWrist.kLastState == RushinatorWrist.State.kScoreL4RightWrist || 
            RushinatorWrist.kLastState == RushinatorWrist.State.kScoreL3RightWrist || 
            RushinatorWrist.kLastState == RushinatorWrist.State.kScoreL2RightWrist || 
            RushinatorWrist.kLastState == RushinatorWrist.State.kTravelL4Right ||
            RushinatorWrist.kLastState == RushinatorWrist.State.kGroundMid || 
            RushinatorWrist.kLastState == RushinatorWrist.State.kScoreL1Mid ||
            RushinatorWrist.kLastState == RushinatorWrist.State.kHPMid)
        );

        controller.povRight().whileTrue(mLineupMaster.directDriveToNearestRightBranch());
        controller.povRight().whileTrue(new ConditionalCommand(
            RobotCommands.coralPrimeAutoScore(RushinatorPivot.State.kStowTravel, RushinatorWrist.State.kTravelRight, ElevatorSubsystem.State.kCoralL4), 
            RobotCommands.coralPrimeAutoScore(RushinatorPivot.State.kStowTravel, RushinatorWrist.State.kTravelLeft, ElevatorSubsystem.State.kCoralL4), 
            () -> RushinatorWrist.kLastState == RushinatorWrist.State.kTravelRight || 
            RushinatorWrist.kLastState == RushinatorWrist.State.kScoreRightWrist || 
            RushinatorWrist.kLastState == RushinatorWrist.State.kScoreL4RightWrist || 
            RushinatorWrist.kLastState == RushinatorWrist.State.kScoreL3RightWrist || 
            RushinatorWrist.kLastState == RushinatorWrist.State.kScoreL2RightWrist || 
            RushinatorWrist.kLastState == RushinatorWrist.State.kTravelL4Right ||
            RushinatorWrist.kLastState == RushinatorWrist.State.kGroundMid || 
            RushinatorWrist.kLastState == RushinatorWrist.State.kScoreL1Mid ||
            RushinatorWrist.kLastState == RushinatorWrist.State.kHPMid)
        );

        /* L4 Scoring */
        controller.a().onTrue(new ConditionalCommand(
            RobotCommands.scoreCoralAutoL4RightWrist(), 
            RobotCommands.scoreCoralAutoL4LeftWrist(), 
            () -> RushinatorWrist.kLastState == RushinatorWrist.State.kTravelRight || 
            RushinatorWrist.kLastState == RushinatorWrist.State.kScoreRightWrist || 
            RushinatorWrist.kLastState == RushinatorWrist.State.kScoreL4RightWrist || 
            RushinatorWrist.kLastState == RushinatorWrist.State.kScoreL3RightWrist || 
            RushinatorWrist.kLastState == RushinatorWrist.State.kScoreL2RightWrist || 
            RushinatorWrist.kLastState == RushinatorWrist.State.kTravelL4Right ||
            RushinatorWrist.kLastState == RushinatorWrist.State.kGroundMid || 
            RushinatorWrist.kLastState == RushinatorWrist.State.kScoreL1Mid ||
            RushinatorWrist.kLastState == RushinatorWrist.State.kHPMid)
        );

        /* L3 Auto Aligning */
        // controller.povUp().whileTrue(mLineupMaster.directDriveToNearestRightBranch());
        // controller.povDown().whileTrue(mLineupMaster.directDriveToNearestLeftBranch());

        // controller.povLeft().and(OperatorXbox.getInstance().leftTriggerOnly()).whileTrue(mLineupMaster.directDriveToNearestLeftBranch());
        // controller.povLeft().and(OperatorXbox.getInstance().leftTriggerOnly()).whileTrue(new ConditionalCommand(
        //     RobotCommands.coralPrimeAutoScore(RushinatorPivot.State.kStowTravel, RushinatorWrist.State.kTravelRight, ElevatorSubsystem.State.kCoralL3), 
        //     RobotCommands.coralPrimeAutoScore(RushinatorPivot.State.kStowTravel, RushinatorWrist.State.kTravelLeft, ElevatorSubsystem.State.kCoralL3), 
        //     () -> RushinatorWrist.kLastState == RushinatorWrist.State.kTravelRight || 
        //     RushinatorWrist.kLastState == RushinatorWrist.State.kScoreRightWrist || 
        //     RushinatorWrist.kLastState == RushinatorWrist.State.kScoreL4RightWrist || 
        //     RushinatorWrist.kLastState == RushinatorWrist.State.kScoreL3RightWrist || 
        //     RushinatorWrist.kLastState == RushinatorWrist.State.kScoreL2RightWrist || 
        //     RushinatorWrist.kLastState == RushinatorWrist.State.kTravelL4Right ||
        //     RushinatorWrist.kLastState == RushinatorWrist.State.kGroundMid || 
        //     RushinatorWrist.kLastState == RushinatorWrist.State.kScoreL1Mid ||
        //     RushinatorWrist.kLastState == RushinatorWrist.State.kHPMid)
        // );

        // controller.povRight().and(OperatorXbox.getInstance().leftTriggerOnly()).whileTrue(mLineupMaster.directDriveToNearestRightBranch());
        // controller.povRight().and(OperatorXbox.getInstance().leftTriggerOnly()).whileTrue(new ConditionalCommand(
        //     RobotCommands.coralPrimeAutoScore(RushinatorPivot.State.kStowTravel, RushinatorWrist.State.kTravelRight, ElevatorSubsystem.State.kCoralL3), 
        //     RobotCommands.coralPrimeAutoScore(RushinatorPivot.State.kStowTravel, RushinatorWrist.State.kTravelLeft, ElevatorSubsystem.State.kCoralL3), 
        //     () -> RushinatorWrist.kLastState == RushinatorWrist.State.kTravelRight || 
        //     RushinatorWrist.kLastState == RushinatorWrist.State.kScoreRightWrist || 
        //     RushinatorWrist.kLastState == RushinatorWrist.State.kScoreL4RightWrist || 
        //     RushinatorWrist.kLastState == RushinatorWrist.State.kScoreL3RightWrist || 
        //     RushinatorWrist.kLastState == RushinatorWrist.State.kScoreL2RightWrist || 
        //     RushinatorWrist.kLastState == RushinatorWrist.State.kTravelL4Right ||
        //     RushinatorWrist.kLastState == RushinatorWrist.State.kGroundMid || 
        //     RushinatorWrist.kLastState == RushinatorWrist.State.kScoreL1Mid ||
        //     RushinatorWrist.kLastState == RushinatorWrist.State.kHPMid)
        // );

        /* L3 Scoring */
        // controller.a().and(OperatorXbox.getInstance().leftTriggerOnly()).onTrue(new ConditionalCommand(
        //     RobotCommands.scoreCoralAutoL3RightWrist(), 
        //     RobotCommands.scoreCoralAutoL3LeftWrist(), 
        //     () -> RushinatorWrist.kLastState == RushinatorWrist.State.kTravelRight || 
        //     RushinatorWrist.kLastState == RushinatorWrist.State.kScoreRightWrist || 
        //     RushinatorWrist.kLastState == RushinatorWrist.State.kScoreL4RightWrist || 
        //     RushinatorWrist.kLastState == RushinatorWrist.State.kScoreL3RightWrist || 
        //     RushinatorWrist.kLastState == RushinatorWrist.State.kScoreL2RightWrist || 
        //     RushinatorWrist.kLastState == RushinatorWrist.State.kTravelL4Right ||
        //     RushinatorWrist.kLastState == RushinatorWrist.State.kGroundMid || 
        //     RushinatorWrist.kLastState == RushinatorWrist.State.kScoreL1Mid ||
        //     RushinatorWrist.kLastState == RushinatorWrist.State.kHPMid)
        // );

        // controller.povUp().onTrue(new AutoAlignHP(() -> HPStation.BLU_RIGHT_STATION.AprilTag));
        // controller.povDown().onTrue(new AutoAlignHP(() -> HPStation.BLU_LEFT_STATION.AprilTag));

        //Align to Reef
        // controller.povLeft().whileTrue(new DriveToPoseCommand(
        //     CommandSwerveDrivetrain.getInstance(), 
        //     () -> PoseEstimatorSubsystem.getInstance().getCurrentPose(), 
        //     Cblue));
        // controller.povRight().whileTrue(new DriveToPoseCommand(CommandSwerveDrivetrain.getInstance(), 
        //     () -> PoseEstimatorSubsystem.getInstance().getCurrentPose(), 
        //     Dblue));


        // // Toggle Wrist Left and Right
        // controller.povLeft().onTrue(new ConditionalCommand(
        //     new SetWristState(RushinatorWrist.State.kTravelLeft), 
        //     new SetWristState(RushinatorWrist.State.kGroundMid), 
        //     () -> RushinatorPivot.kLastState != RushinatorPivot.State.kFloorIntake)
        // );

        // controller.povRight().onTrue(new ConditionalCommand(
        //     new SetWristState(RushinatorWrist.State.kTravelRight), 
        //     new SetWristState(RushinatorWrist.State.kGroundMid), 
        //     () -> RushinatorPivot.kLastState != RushinatorPivot.State.kFloorIntake)
        // );

        // Zero Elevator
        controller.leftBumper().onTrue(new SetElevatorState(ElevatorSubsystem.State.kZero));

        // Zero Drivetrain
        controller.start().onTrue(CommandSwerveDrivetrain.getInstance().runOnce(()-> CommandSwerveDrivetrain.getInstance().seedFieldCentric()));

        /*TESTING BINDINGS */

        // controller.povUp().onTrue(new SetClimberAngle(Climber.State.kDeploy.pos));
        // controller.povDown().onTrue(new SetClimberAngle(Climber.State.kRetract.pos));
        // controller.povLeft().onTrue(new SetClimberAngle(Climber.State.kStow.pos));

        // controller.povLeft().whileTrue(new AutoAlign(true));
        // controller.povRight().whileTrue(new AutoAlign(false));


        // controller.povUp().onTrue(RobotCommands.scoreCoralAutoL4RightWrist());
        // controller.povDown().onTrue(RobotCommands.scoreCoralAutoL3RightWrist());
        // controller.povRight().onTrue(RobotCommands.scoreCoralAutonL1());

        /*Elevator TEst Commands */
        // controller.povDown().onTrue(new SetElevatorState(ElevatorSubsystem.State.kZero));
        // controller.povLeft().onTrue(new SetElevatorState(ElevatorSubsystem.State.kAlgaeIntake));
        // controller.povRight().onTrue(new SetElevatorState(ElevatorSubsystem.State.kAlgaeL3));
        // controller.povUp().onTrue(new SetElevatorState(ElevatorSubsystem.State.kAlgaeL3));

        // controller.povLeft().whileTrue(new LineupCommand(true));
        // controller.povRight().whileTrue(new LineupCommand(false));

        // controller.rightTrigger().whileTrue(AlgaePivotCommands.setAlgaePivotA    ngle(AlgaeSubsystem.State.kFloorIntake));
        // controller.rightTrigger().whileTrue(new AlgaeRoller.IntakeCommand());
        // controller.leftTrigger().whileTrue(new CoralRollerSubsystem.SetVoltageCommand(12));

        // controller.rightTrigger().whileTrue(new InstantCommand(() -> RobotContainer.modeFast = false));
        // controller.rightTrigger().whileFalse(new InstantCommand(() -> RobotContainer.modeFast = true));
        // controller.leftBumper().onTrue(new LineupCommand());

        /*Algae Pivot TEsting */
        // controller.a().onTrue(AlgaePivotCommands.setAlgaePivotAngle(AlgaeSubsystem.State.kFloorIntake));
        // controller.b().onTrue(AlgaePivotCommands.setAlgaePivotAngle(AlgaeSubsystem.State.kProcessor));
        // controller.x().onTrue(AlgaePivotCommands.setAlgaePivotAngle(AlgaeSubsystem.State.kReefIntake));
        // controller.y().onTrue(AlgaePivotCommands.setAlgaePivotAngle(AlgaeSubsystem.State.kScore));

        /*Coral Arm Pivot TEsting */
        // controller.y().onTrue(new SetArmState(RushinatorPivot.State.kTestPos));
        // controller.a().onTrue(new SetArmState(RushinatorPivot.State.kTestPos2));

        // controller.x().onTrue(RobotCommands.coralPrime(
        //     RushinatorPivot.State.kStowTravel, ElevatorSubsystem.State.kZero, RushinatorWrist.State.kTravelRight)
        // );

        // controller.povUp().onTrue(RobotCommands.coralPrime(
        //     RushinatorPivot.State.kScore, ElevatorSubsystem.State.kZero, RushinatorWrist.State.kTravelMid)
        // );

        // controller.b().onTrue(RobotCommands.coralPrime(
        //     RushinatorPivot.State.kStowTravel, ElevatorSubsystem.State.kCoralL3, RushinatorWrist.State.kTravelRight)
        // );

        // controller.y().onTrue(RobotCommands.coralPrime(
        //     RushinatorPivot.State.kStowTravel, ElevatorSubsystem.State.kCoralL4, RushinatorWrist.State.kTravelRight)
        // );

        // controller.rightBumper().whileTrue(new ConditionalCommand(
        //     RobotCommands.coralPrimeShoot(RushinatorPivot.State.kScore, RushinatorWrist.State.kScoreRightWrist), 
        //     RobotCommands.coralPrimeShoot(RushinatorPivot.State.kScore, RushinatorWrist.State.kScoreLeftWrist), 
        //     () -> RushinatorWrist.kLastState == RushinatorWrist.State.kTravelRight || 
        //     RushinatorWrist.kLastState == RushinatorWrist.State.kScoreRightWrist || 
        //     RushinatorWrist.kLastState == RushinatorWrist.State.kGroundMid || 
        //     RushinatorWrist.kLastState == RushinatorWrist.State.kHPMid)
        // );
        // controller.rightBumper().whileTrue(new SetRollersVoltage(0.0));

        // controller.a().whileTrue(RobotCommands.coralPrime(
        //     RushinatorPivot.State.kFloorIntake, ElevatorSubsystem.State.kZero, RushinatorWrist.State.kGroundMid)
        // );
        // controller.a().whileTrue(new SetRollersVoltage(4.0));

        // controller.leftBumper().whileTrue(new SetRollersVoltage(-1.5));

        // controller.b().onTrue(new ToggleWristState());
        // controller.povLeft().onTrue(new ConditionalCommand(
        //     new SetWristState(RushinatorWrist.State.kTravelLeft), 
        //     new SetWristState(RushinatorWrist.State.kGroundMid), 
        //     () -> RushinatorPivot.kLastState != RushinatorPivot.State.kFloorIntake)
        // );

        // controller.povRight().onTrue(new ConditionalCommand(
        //     new SetWristState(RushinatorWrist.State.kTravelRight), 
        //     new SetWristState(RushinatorWrist.State.kGroundMid), 
        //     () -> RushinatorPivot.kLastState != RushinatorPivot.State.kFloorIntake)
        // );
        
        /*Wrist TEsting*/
        // controller.povLeft().onTrue(new SetWristState(RushinatorWrist.State.kTravelLeft));
        // controller.povUp().onTrue(new SetWristState(RushinatorWrist.State.kTravelMid));
        // controller.povRight().onTrue(new SetWristState(RushinatorWrist.State.kTravelRight));

        // controller.a().onTrue(new SetArmState(RushinatorPivot.State.kTestPos));
        
        // controller.leftBumper().onTrue(AutonMaster.getInstance().getTestPathFindingCommand());

        // controller.y().onTrue(new InstantCommand(() -> CommandSwerveDrivetrain.getInstance().zeroHeading()));
        // controller.leftTrigger().onTrue(CommandSwerveDrivetrain.getInstance().runOnce(()-> CommandSwerveDrivetrain.getInstance().seedFieldCentric()));
    }

    @Override
    public void setupDisabledButtons() {}

    @Override
    public void setupTestButtons() {}

    public Translation2d getDriveTranslation() {
        final var xComponent = translationStickCurve.calculate(controller.getLeftX());
        final var yComponent = translationStickCurve.calculate(controller.getLeftY());
        // Components are reversed because field coordinates are opposite of joystick coordinates
        return new Translation2d(yComponent, xComponent);
    }

    public void setDriveRotation(double requestedAngularVel) {
        reqAngularVel = requestedAngularVel;
    }

    public double getDriveRotation() {
        return rotationStickCurve.calculate(-controller.getRightX());
    }

    public boolean isLeftPovPressed() {
        return controller.povLeft().getAsBoolean();
    }

    public boolean isRightPovPressed() {
        return controller.povRight().getAsBoolean();
    }

    public double getRightX() { return controller.getRightX(); }
}