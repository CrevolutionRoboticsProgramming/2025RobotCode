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
import frc.robot.algaepivot.AlgaeSubsystem;
import frc.robot.algaepivot.commands.AlgaePivotCommands;
import frc.robot.algaepivot.commands.SetAngleAlgaePivot;
import frc.robot.commands.RobotCommands;
import frc.robot.coralArm.CoralSubsystem;
import frc.robot.coralator.CoralRollerSubsystem;
import frc.robot.elevator.ElevatorSubsystem;
import frc.robot.elevator.ElevatorSubsystem.State;
import frc.robot.indexer.commands.IndexerCommands;

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
