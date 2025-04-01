package frc.robot.operator;

import com.ctre.phoenix6.controls.ControlRequest;

import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.crevolib.util.ExpCurve;
import frc.crevolib.util.Gamepad;
import frc.crevolib.util.XboxGamepad;
import frc.robot.Robot;
import frc.robot.algaeflywheel.AlgaeRoller;
import frc.robot.algaeflywheel.commands.AlgaeRollerCommands;
import frc.robot.algaepivot.AlgaeSubsystem;
import frc.robot.algaepivot.commands.AlgaePivotCommands;
import frc.robot.algaepivot.commands.SetAngleAlgaePivot;
import frc.robot.climber.Climber;
import frc.robot.climber.commands.SetClimberAngle;
import frc.robot.commands.RobotCommands;
import frc.robot.driver.DriverXbox;
import frc.robot.elevator.ElevatorSubsystem;
import frc.robot.elevator.ElevatorSubsystem.State;
import frc.robot.elevator.commands.SetElevatorState;
import frc.robot.indexer.commands.IndexerCommands;
import frc.robot.rushinator.RushinatorPivot;
import frc.robot.rushinator.RushinatorWrist;
import frc.robot.rushinator.commands.SetArmState;
import frc.robot.rushinator.commands.SetRollersVoltage;
import frc.robot.rushinator.commands.SetWristState;

public class OperatorXbox extends XboxGamepad {
    private static class Settings {
        static final int port = 1;
        static final String name = "operator";

        static final double kDeadzone = 0.1;
    }

    ExpCurve stickCurve;
    ExpCurve shooterPivotManualCurve;
    ExpCurve intakePivotManualCurve;
    ExpCurve positionTestCurve;
    ExpCurve elevatorCurve;
    private static OperatorXbox mInstance;

    ExpCurve shooterManualCurve;

    private OperatorXbox() {
        super(Settings.name, Settings.port);

        stickCurve = new ExpCurve(1, 0, 1, Settings.kDeadzone);
    }

    public static OperatorXbox getInstance() {
        if (mInstance == null) {
            mInstance = new OperatorXbox();
        }
        return mInstance;
    }

    @Override
    public void setupTeleopButtons() {
        /*Comp Bindings */

        // Algae Reef Intake
        controller.leftBumper().and(leftTriggerOnly()).whileTrue(new AlgaeRoller.IntakeCommand());
        controller.leftBumper().and(leftTriggerOnly()).whileTrue(new SetAngleAlgaePivot(AlgaeSubsystem.State.kReefIntake));

        // Jogging the ELevator Up and Down
        // controller.povUp().whileTrue(new ElevatorSubsystem.applyJog(ElevatorSubsystem.getInstance().getPosition() + 5.0));
        // controller.povDown().whileTrue(new ElevatorSubsystem.applyJog(ElevatorSubsystem.getInstance().getPosition() - 3.0));

        // Algae (Elevator) Barge Shot 
        controller.leftBumper().onTrue(new SetElevatorState(ElevatorSubsystem.State.kCoralL4));
        controller.leftBumper().onTrue(new SetAngleAlgaePivot(AlgaeSubsystem.State.kStow));
        controller.leftBumper().onTrue(new AlgaeRoller.PrimeCommand());

        // Primes the processor shooting
        controller.rightTrigger().whileTrue(new SetAngleAlgaePivot(AlgaeSubsystem.State.kProcessor));
        controller.rightTrigger().whileTrue(new SetElevatorState(ElevatorSubsystem.State.kZero));

        // ALgae Scooring For Far Barge Shot
        controller.rightBumper().and(() -> DriverXbox.getInstance().controller.a().getAsBoolean()).whileTrue(new AlgaeRoller.FarShootCommand());

        // Algae SCorring for Process & Barge
        controller.rightBumper().and(leftTriggerOnly()).whileTrue(new AlgaeRoller.ProcessShootCommand());
        controller.rightBumper().whileTrue(new AlgaeRoller.ShootCommand());

        // ADjusting Coral ORinetaiton
        controller.povLeft().onTrue(new ConditionalCommand(
            new SetWristState(RushinatorWrist.State.kTravelLeft), 
            new SetWristState(RushinatorWrist.State.kTravelL4Left), 
            () -> RushinatorWrist.kLastState == RushinatorWrist.State.kTravelRight
            )
        );

        controller.povRight().onTrue(new ConditionalCommand(
            new SetWristState(RushinatorWrist.State.kTravelRight), 
            new SetWristState(RushinatorWrist.State.kTravelL4Right), 
            () -> RushinatorWrist.kLastState == RushinatorWrist.State.kTravelRight
            )
        );

        // controller.povDown().whileTrue(RobotCommands.coralPrime(
        //     RushinatorPivot.State.kLoliPop, ElevatorSubsystem.State.kZero)
        // );
        // controller.povDown().whileTrue(
        //     new SetWristState(RushinatorWrist.State.kLoliRight)
        // );
        // controller.povDown().whileTrue(new SetRollersVoltage(4.5));

        controller.povUp().onTrue(RobotCommands.scoreCoralAutonL4());
        controller.povDown().onTrue(RobotCommands.autoHPPickUp());

        // controller.povUp().and(leftTriggerOnly()).onTrue(new SetAngleAlgaePivot(AlgaeSubsystem.State.kClimb));
        // controller.povUp().and(leftTriggerOnly()).onTrue(new SetArmState(RushinatorPivot.State.kClimb));
        // controller.povUp().and(leftTriggerOnly()).onTrue(new SetWristState(RushinatorWrist.State.kClimblRight));
        // controller.povUp().and(leftTriggerOnly()).onTrue(new SetElevatorState(ElevatorSubsystem.State.kClimb));
        // // controller.povUp().and(leftTriggerOnly()).onTrue(new SetClimberAngle(Climber.State.kDeploy.pos));

        // controller.povDown().and(leftTriggerOnly()).onTrue(new SetAngleAlgaePivot(AlgaeSubsystem.State.kClimb));
        // controller.povDown().and(leftTriggerOnly()).onTrue(new SetElevatorState(ElevatorSubsystem.State.kClimb));
        // controller.povDown().and(leftTriggerOnly()).onTrue(new SetArmState(RushinatorPivot.State.kClimb));
        // controller.povDown().and(leftTriggerOnly()).onTrue(new SetWristState(RushinatorWrist.State.kClimblRight));
        // // controller.povDown().and(leftTriggerOnly()).onTrue(new SetClimberAngle(Climber.State.kRetract.pos));

        // Score Prime L1
        controller.a().whileTrue(RobotCommands.coralPrime(
            RushinatorPivot.State.kScoreL1, ElevatorSubsystem.State.kZero)
        );
        controller.a().onTrue(new ConditionalCommand(
            new SetWristState(RushinatorWrist.State.kScoreL1Mid), 
            new SetWristState(RushinatorWrist.State.kScoreL1Mid), 
            () -> RushinatorWrist.kLastState == RushinatorWrist.State.kTravelRight || 
            RushinatorWrist.kLastState == RushinatorWrist.State.kGroundMid ||
            RushinatorWrist.kLastState == RushinatorWrist.State.kHPMid || 
            RushinatorWrist.kLastState == RushinatorWrist.State.kScoreL4RightWrist || 
            RushinatorWrist.kLastState == RushinatorWrist.State.kScoreL3RightWrist || 
            RushinatorWrist.kLastState == RushinatorWrist.State.kScoreL2RightWrist || 
            RushinatorWrist.kLastState == RushinatorWrist.State.kTravelL4Right ||
            RushinatorWrist.kLastState == RushinatorWrist.State.kScoreL1Mid 
            )
        );

        // Score Prime L2
        controller.x().whileTrue(RobotCommands.coralPrime(
            RushinatorPivot.State.kStowTravel, ElevatorSubsystem.State.kCoralL2)
        );
        controller.x().onTrue(new ConditionalCommand(
            new SetWristState(RushinatorWrist.State.kTravelRight), 
            new SetWristState(RushinatorWrist.State.kTravelLeft), 
            () -> RushinatorWrist.kLastState == RushinatorWrist.State.kTravelRight ||
            RushinatorWrist.kLastState == RushinatorWrist.State.kGroundMid ||
            RushinatorWrist.kLastState == RushinatorWrist.State.kHPMid || 
            RushinatorWrist.kLastState == RushinatorWrist.State.kScoreL4RightWrist || 
            RushinatorWrist.kLastState == RushinatorWrist.State.kScoreL3RightWrist ||
            RushinatorWrist.kLastState == RushinatorWrist.State.kTravelL4Right || 
            RushinatorWrist.kLastState == RushinatorWrist.State.kScoreL2RightWrist || 
            RushinatorWrist.kLastState == RushinatorWrist.State.kScoreL1Mid 
            )
        );

        // Score Prime L3
        controller.b().onTrue(RobotCommands.coralPrime(
            RushinatorPivot.State.kStowTravel, ElevatorSubsystem.State.kCoralL3)
        );
        controller.b().onTrue(new ConditionalCommand(
            new SetWristState(RushinatorWrist.State.kTravelRight), 
            new SetWristState(RushinatorWrist.State.kTravelLeft), 
            () -> RushinatorWrist.kLastState == RushinatorWrist.State.kTravelRight ||
            RushinatorWrist.kLastState == RushinatorWrist.State.kGroundMid ||
            RushinatorWrist.kLastState == RushinatorWrist.State.kHPMid || 
            RushinatorWrist.kLastState == RushinatorWrist.State.kTravelL4Right ||
            RushinatorWrist.kLastState == RushinatorWrist.State.kScoreL4RightWrist || 
            RushinatorWrist.kLastState == RushinatorWrist.State.kScoreL3RightWrist || 
            RushinatorWrist.kLastState == RushinatorWrist.State.kScoreL2RightWrist || 
            RushinatorWrist.kLastState == RushinatorWrist.State.kScoreL1Mid
            )
        );

        // Score Prime L4
        controller.y().onTrue(RobotCommands.coralPrime(
            RushinatorPivot.State.kStowL4, ElevatorSubsystem.State.kCoralL4)
        );
        controller.y().onTrue(new ConditionalCommand(
            new SetWristState(RushinatorWrist.State.kTravelL4Right), 
            new SetWristState(RushinatorWrist.State.kTravelL4Left), 
            () -> RushinatorWrist.kLastState == RushinatorWrist.State.kTravelRight ||
            RushinatorWrist.kLastState == RushinatorWrist.State.kTravelL4Right ||
            RushinatorWrist.kLastState == RushinatorWrist.State.kScoreL4RightWrist || 
            RushinatorWrist.kLastState == RushinatorWrist.State.kScoreL3RightWrist || 
            RushinatorWrist.kLastState == RushinatorWrist.State.kScoreL2RightWrist || 
            RushinatorWrist.kLastState == RushinatorWrist.State.kScoreL1Mid ||
            RushinatorWrist.kLastState == RushinatorWrist.State.kGroundMid ||
            RushinatorWrist.kLastState == RushinatorWrist.State.kHPMid
            )
        );

        // Algae L3
        controller.y().and(leftTriggerOnly()).onTrue(new SetElevatorState(ElevatorSubsystem.State.kAlgaeL3));
        controller.y().and(leftTriggerOnly()).whileTrue(new AlgaeRoller.IntakeCommand());
        controller.y().and(leftTriggerOnly()).whileTrue(new SetAngleAlgaePivot(AlgaeSubsystem.State.kReefIntake));

        // Algae L2 
        controller.b().and(leftTriggerOnly()).onTrue(new SetElevatorState(ElevatorSubsystem.State.kAlgaeL2));
        controller.b().and(leftTriggerOnly()).whileTrue(new AlgaeRoller.IntakeCommand());
        controller.b().and(leftTriggerOnly()).whileTrue(new SetAngleAlgaePivot(AlgaeSubsystem.State.kReefIntake));

        /*TEsting Bindings */

//        controller.leftBumper().whileTrue(IndexerCommands.setOutput(() -> -1.0));
//
//        controller.leftTrigger().whileTrue(RobotCommands.primeShoot());
//        controller.leftTrigger().whileTrue(IndexerCommands.setOutput(() -> 1.0));
//        controller.leftBumper().whileTrue(new AlgaeRoller.ShootCommand());

/*Used Commands */
        // controller.leftTrigger().whileTrue(new SetAngleAlgaePivot(AlgaeSubsystem.State.kFloorIntake));
        // controller.leftTrigger().whileTrue(new AlgaeRoller.PrimeCommand());
        // controller.leftBumper().whileTrue(new AlgaeRoller.ShootCommand());

//
        // controller.rightTrigger().whileTrue(new CoralSubsystem.SetStateCommand(CoralSubsystem.State.kScoreV2));
        // controller.rightBumper().whileTrue(new CoralRollerSubsystem.SetVoltageCommand(-12));

        // controller.x().whileTrue(new AlgaeRoller.ProcessShootCommand());

        /*Didn't use these commands */
        // controller.y().whileTrue(new SetAngleAlgaePivot(AlgaeSubsystem.State.kStow));
        // controller.y().whileTrue(new AlgaeRoller.PrimeCommand());

        // controller.povLeft().onTrue(new InstantCommand(() -> ElevatorSubsystem.getInstance().setTargetState(ElevatorSubsystem.State.kCoralL1)));
        // controller.povUp().onTrue(new InstantCommand(() -> ElevatorSubsystem.getInstance().setTargetState(ElevatorSubsystem.State.kCoralL2)));
        // controller.povRight().onTrue(new InstantCommand(() -> ElevatorSubsystem.getInstance().setTargetState(ElevatorSubsystem.State.kCoralL3)));

        // controller.a().onTrue(new InstantCommand(() -> ElevatorSubsystem.getInstance().setTargetState(ElevatorSubsystem.State.kAlgaeL2)));
        // controller.b().onTrue(new InstantCommand(() -> ElevatorSubsystem.getInstance().setTargetState(ElevatorSubsystem.State.kAlgaeL3)));


        // controller.povLeft().whileTrue(RobotCommands.coralPrime(CoralSubsys  tem.State.kScoreV2, ElevatorSubsystem.State.kCoralL1));
        // controller.povUp().whileTrue(RobotCommands.coralPrime(CoralSubsystem.State.kScoreV2, ElevatorSubsystem.State.kCoralL2));
        // controller.povRight().whileTrue(RobotCommands.coralPrime(CoralSubsystem.State.kScoreV2, ElevatorSubsystem.State.kCoralL3));

        // controller.a().whileTrue(RobotCommands.algaePrime(AlgaeSubsystem.State.kFloorIntake, ElevatorSubsystem.State.kAlgaeL2));
        // controller.b().whileTrue(RobotCommands.algaePrime(AlgaeSubsystem.State.kFloorIntake, ElevatorSubsystem.State.kAlgaeL3));
    }

    @Override
    public void setupDisabledButtons() {

    }

    @Override
    public void setupTestButtons() {
    }



    public Translation2d getDriveTranslation() {
        final var xComponent = stickCurve.calculate(controller.getLeftX());
        final var yComponent = stickCurve.calculate(controller.getLeftY());
        // Components are reversed because field coordinates are opposite of joystick coordinates
        return new Translation2d(yComponent, xComponent);
    }

    public double getElevatorOutput() {
        return stickCurve.calculate(-controller.getRightY());
    }

    public double getDriveRotation() {
        return -stickCurve.calculate(-controller.getRightX());
    }

    public double getLeftY() { 
        return controller.getLeftY();
    }
}
