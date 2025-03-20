package frc.robot.driver;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.crevolib.util.ExpCurve;
import frc.crevolib.util.XboxGamepad;
import frc.robot.RobotContainer;
import frc.robot.algaeflywheel.AlgaeRoller;
import frc.robot.algaepivot.AlgaeSubsystem;
import frc.robot.algaepivot.commands.AlgaePivotCommands;
import frc.robot.auton.AutonMaster;
import frc.robot.coralator.CoralRollerSubsystem;
import frc.robot.drivetrain.CommandSwerveDrivetrain;
import frc.robot.indexer.commands.IndexerCommands;
import frc.robot.rushinator.RushinatorPivot;
import frc.robot.rushinator.RushinatorWrist;
import frc.robot.rushinator.commands.SetArmState;
import frc.robot.rushinator.commands.SetWristState;
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
        // controller.rightTrigger().whileTrue(AlgaePivotCommands.setAlgaePivotAngle(AlgaeSubsystem.State.kFloorIntake));
        // controller.rightTrigger().whileTrue(new AlgaeRoller.IntakeCommand());
        // controller.leftTrigger().whileTrue(new CoralRollerSubsystem.SetVoltageCommand(12));

        controller.rightBumper().whileTrue(new InstantCommand(() -> RobotContainer.modeFast = false));
        controller.rightBumper().whileFalse(new InstantCommand(() -> RobotContainer.modeFast = true));
        controller.leftBumper().onTrue(new LineupCommand());

        /*Algae Pivot TEsting */
        // controller.a().onTrue(AlgaePivotCommands.setAlgaePivotAngle(AlgaeSubsystem.State.kFloorIntake));
        // controller.b().onTrue(AlgaePivotCommands.setAlgaePivotAngle(AlgaeSubsystem.State.kProcessor));
        // controller.x().onTrue(AlgaePivotCommands.setAlgaePivotAngle(AlgaeSubsystem.State.kReefIntake));
        // controller.y().onTrue(AlgaePivotCommands.setAlgaePivotAngle(AlgaeSubsystem.State.kScore));

        /*Coral Arm Pivot TEsting */
        controller.y().onTrue(new SetArmState(RushinatorPivot.State.kStowTravel));
        controller.a().onTrue(new SetArmState(RushinatorPivot.State.kTestPos));

        /*Wrist TEsting*/
        // controller.x().onTrue(new SetWristState(RushinatorWrist.State.kScoreLeftWrist));
        // controller.y().onTrue(new SetWristState(RushinatorWrist.State.kPickUp));
        // controller.b().onTrue(new SetWristState(RushinatorWrist.State.kScoreRightWrist));

        // controller.a().onTrue(new SetArmState(RushinatorPivot.State.kTestPos));
        
        // controller.leftBumper().onTrue(AutonMaster.getInstance().getTestPathFindingCommand());

        // controller.y().onTrue(new InstantCommand(() -> CommandSwerveDrivetrain.getInstance().zeroHeading()));
        controller.y().onTrue(CommandSwerveDrivetrain.getInstance().runOnce(()-> CommandSwerveDrivetrain.getInstance().seedFieldCentric()));
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