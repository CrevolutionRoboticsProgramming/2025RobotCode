package frc.robot.driver;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.crevolib.util.ExpCurve;
import frc.crevolib.util.XboxGamepad;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.algaeflywheel.AlgaeRoller;
import frc.robot.algaepivot.AlgaeSubsystem;
import frc.robot.algaepivot.commands.AlgaePivotCommands;
import frc.robot.algaepivot.commands.SetAngleAlgaePivot;
import frc.robot.auton.AutonMaster;
import frc.robot.commands.RobotCommands;
import frc.robot.coralator.CoralRollerSubsystem;
import frc.robot.drivetrain.CommandSwerveDrivetrain;
import frc.robot.elevator.ElevatorSubsystem;
import frc.robot.elevator.commands.ElevatorCommands;
import frc.robot.elevator.commands.SetElevatorState;
import frc.robot.indexer.commands.IndexerCommands;
import frc.robot.rushinator.RushinatorPivot;
import frc.robot.rushinator.RushinatorWrist;
import frc.robot.rushinator.commands.SetArmState;
import frc.robot.rushinator.commands.SetWristState;
import frc.robot.rushinator.commands.ToggleWristState;
import frc.robot.rushinator.commands.SetRollersVoltage;
import frc.robot.vision.commands.LineupCommand;


public class DriverXbox extends XboxGamepad {
    private static class Settings {
        static final int port = 0;
        static final String name = "driver";

        static final double kTranslationExpVal = 2.0;
        static final double kRotationExpVal = 1.0;
        static final double kDeadzone = 0.1;
    }

    private static DriverXbox mInstance;
    public static ExpCurve translationStickCurve;
    private static ExpCurve rotationStickCurve;
    public boolean autoAim;
    private double reqAngularVel;
    private static AlgaeSubsystem mAlgaeSubsystem;
    

    private DriverXbox() {
        super(DriverXbox.Settings.name, DriverXbox.Settings.port);

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
        controller.y().whileTrue(new SetRollersVoltage(3.5));

        //Score Coral
        controller.a().whileTrue(new ConditionalCommand(
            RobotCommands.coralPrimeShoot(RushinatorPivot.State.kScore, RushinatorWrist.State.kScoreRightWrist), 
            RobotCommands.coralPrimeShoot(RushinatorPivot.State.kScore, RushinatorWrist.State.kScoreLeftWrist), 
            () -> RushinatorWrist.kLastState == RushinatorWrist.State.kTravelRight || 
            RushinatorWrist.kLastState == RushinatorWrist.State.kScoreRightWrist || 
            RushinatorWrist.kLastState == RushinatorWrist.State.kGroundMid || 
            RushinatorWrist.kLastState == RushinatorWrist.State.kHPMid)
        );

        // Pulse Alage
        controller.x().whileTrue(new AlgaeRoller.IntakeCommand());

        // Spit Algae
        controller.b().whileTrue(new AlgaeRoller.ShootCommand());

        // Coral Ground Intake
        controller.rightTrigger().whileTrue(RobotCommands.coralPrime(
            RushinatorPivot.State.kFloorIntake, ElevatorSubsystem.State.kZero, RushinatorWrist.State.kGroundMid)
        );
        controller.rightTrigger().whileTrue(new SetRollersVoltage(4.0));

        // Coral HP Intake
        controller.rightBumper().whileTrue(RobotCommands.coralPrime(
            RushinatorPivot.State.kHPIntake, ElevatorSubsystem.State.kZero, RushinatorWrist.State.kHPMid)
        );
        controller.rightBumper().whileTrue(new SetRollersVoltage(4.0));

        // Algae Ground Intake
        controller.leftTrigger().whileTrue(new AlgaeRoller.IntakeCommand());
        controller.leftTrigger().whileTrue(new SetAngleAlgaePivot(AlgaeSubsystem.State.kFloorIntake));
        controller.leftTrigger().whileTrue(new SetElevatorState(ElevatorSubsystem.State.kAlgaeIntake));

        // Toggle Wrist Left and Right
        controller.povLeft().onTrue(new ConditionalCommand(
            new SetWristState(RushinatorWrist.State.kTravelLeft), 
            new SetWristState(RushinatorWrist.State.kGroundMid), 
            () -> RushinatorPivot.kLastState != RushinatorPivot.State.kFloorIntake)
        );

        controller.povRight().onTrue(new ConditionalCommand(
            new SetWristState(RushinatorWrist.State.kTravelRight), 
            new SetWristState(RushinatorWrist.State.kGroundMid), 
            () -> RushinatorPivot.kLastState != RushinatorPivot.State.kFloorIntake)
        );

        // Zero Elevator
        controller.leftBumper().onTrue(new SetElevatorState(ElevatorSubsystem.State.kZero));

        // Zero Drivetrain
        controller.start().onTrue(CommandSwerveDrivetrain.getInstance().runOnce(()-> CommandSwerveDrivetrain.getInstance().seedFieldCentric()));

        /*TESTING BINDINGS */

        

        /*Elevator TEst Commands */
        // controller.a().onTrue(new SetElevatorState(ElevatorSubsystem.State.kZero));
        // controller.x().onTrue(new SetElevatorState(ElevatorSubsystem.State.kCoralL3));
        // controller.y().onTrue(new SetElevatorState(ElevatorSubsystem.State.kCoralL4));

        // controller.povLeft().whileTrue(new LineupCommand(true));
        // controller.povRight().whileTrue(new LineupCommand(false));

        // controller.rightTrigger().whileTrue(AlgaePivotCommands.setAlgaePivotAngle(AlgaeSubsystem.State.kFloorIntake));
        // controller.rightTrigger().whileTrue(new AlgaeRoller.IntakeCommand());
        // controller.leftTrigger().whileTrue(new CoralRollerSubsystem.SetVoltageCommand(12));

        controller.rightTrigger().whileTrue(new InstantCommand(() -> RobotContainer.modeFast = false));
        controller.rightTrigger().whileFalse(new InstantCommand(() -> RobotContainer.modeFast = true));
        // controller.leftBumper().onTrue(new LineupCommand());

        /*Algae Pivot TEsting */
        // controller.a().onTrue(AlgaePivotCommands.setAlgaePivotAngle(AlgaeSubsystem.State.kFloorIntake));
        // controller.b().onTrue(AlgaePivotCommands.setAlgaePivotAngle(AlgaeSubsystem.State.kProcessor));
        // controller.x().onTrue(AlgaePivotCommands.setAlgaePivotAngle(AlgaeSubsystem.State.kReefIntake));
        // controller.y().onTrue(AlgaePivotCommands.setAlgaePivotAngle(AlgaeSubsystem.State.kScore));

        /*Coral Arm Pivot TEsting */
        // controller.y().onTrue(new SetArmState(RushinatorPivot.State.kTestPos));
        // controller.a().onTrue(new SetArmState(RushinatorPivot.State.kTestPos2));

        controller.x().onTrue(RobotCommands.coralPrime(
            RushinatorPivot.State.kStowTravel, ElevatorSubsystem.State.kZero, RushinatorWrist.State.kTravelRight)
        );

        controller.povUp().onTrue(RobotCommands.coralPrime(
            RushinatorPivot.State.kScore, ElevatorSubsystem.State.kZero, RushinatorWrist.State.kTravelMid)
        );

        controller.b().onTrue(RobotCommands.coralPrime(
            RushinatorPivot.State.kStowTravel, ElevatorSubsystem.State.kCoralL3, RushinatorWrist.State.kTravelRight)
        );

        controller.y().onTrue(RobotCommands.coralPrime(
            RushinatorPivot.State.kStowTravel, ElevatorSubsystem.State.kCoralL4, RushinatorWrist.State.kTravelRight)
        );

        controller.rightBumper().whileTrue(new ConditionalCommand(
            RobotCommands.coralPrimeShoot(RushinatorPivot.State.kScore, RushinatorWrist.State.kScoreRightWrist), 
            RobotCommands.coralPrimeShoot(RushinatorPivot.State.kScore, RushinatorWrist.State.kScoreLeftWrist), 
            () -> RushinatorWrist.kLastState == RushinatorWrist.State.kTravelRight || 
            RushinatorWrist.kLastState == RushinatorWrist.State.kScoreRightWrist || 
            RushinatorWrist.kLastState == RushinatorWrist.State.kGroundMid || 
            RushinatorWrist.kLastState == RushinatorWrist.State.kHPMid)
        );
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
        // controller.povLeft().onTrue(new SetWristState(RushinatorWrist.State.kScoreLeftWrist));
        // controller.povUp().onTrue(new SetWristState(RushinatorWrist.State.kPickUp));
        // controller.povRight().onTrue(new SetWristState(RushinatorWrist.State.kScoreRightWrist));

        // controller.a().onTrue(new SetArmState(RushinatorPivot.State.kTestPos));
        
        // controller.leftBumper().onTrue(AutonMaster.getInstance().getTestPathFindingCommand());

        // controller.y().onTrue(new InstantCommand(() -> CommandSwerveDrivetrain.getInstance().zeroHeading()));
        controller.leftTrigger().onTrue(CommandSwerveDrivetrain.getInstance().runOnce(()-> CommandSwerveDrivetrain.getInstance().seedFieldCentric()));
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

    public double getRightX() { return controller.getRightX(); }
}