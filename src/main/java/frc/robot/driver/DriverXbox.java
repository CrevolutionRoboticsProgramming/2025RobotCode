package frc.robot.driver;

import java.util.Optional;
import java.util.Set;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.crevolib.util.ExpCurve;
import frc.crevolib.util.XboxGamepad;
import frc.robot.Robot;
import frc.robot.algaeflywheel.AlgaeFlyWheel;
import frc.robot.algaeflywheel.commands.AlgaeFlyWheelCommands;
import frc.robot.algaeflywheel.commands.SetVelocityAlgaeFlyWheel;
import frc.robot.algaepivot.AlgaeSubsystem;
import frc.robot.algaepivot.commands.AlgaePivotCommands;
import frc.robot.algaepivot.commands.SetAngleAlgaePivot;
import frc.robot.drivetrain2.Drivetrain;
import frc.robot.indexer.commands.IndexerCommands;


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
        // controller.a().whileTrue(
        //     AlgaeFlyWheelCommands.setAngularVelocity(                
        //     () -> AlgaeFlyWheel.Settings.kMaxAngularVelocity.times(0.8),
        //     AlgaeFlyWheel.Settings.kAlgaeScoringInverted,
        //     AlgaeFlyWheel.Settings.kAlgaeIntakingInverted
        // ));

        // controller.b().whileTrue(IndexerCommands.setOutput(() ->0.10));

        // controller.y().onTrue(new InstantCommand(() -> Drivetrain.getInstance().zeroHeading()));

        
        controller.a().whileTrue(AlgaePivotCommands.setAlgaePivotAngle(AlgaeSubsystem.State.kFloorIntake));
        controller.b().whileTrue(AlgaePivotCommands.setAlgaePivotAngle(AlgaeSubsystem.State.kReefIntake));
        controller.x().whileTrue(AlgaePivotCommands.setAlgaePivotAngle(AlgaeSubsystem.State.kScore));
        controller.y().whileTrue(AlgaePivotCommands.setAlgaePivotAngle(AlgaeSubsystem.State.kStow));

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