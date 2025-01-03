package frc.robot.driver;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.crevolib.util.ExpCurve;
import frc.crevolib.util.Gamepad;
import frc.robot.Robot;
import frc.robot.drivetrain.Drivetrain;

public class Driver extends Gamepad {
    private static class Settings {
        static final int port = 0;
        static final String name = "driver";

        static final double kTranslationExpVal = 2.0;
        static final double kRotationExpVal = 1.0;
        static final double kDeadzone = 0.1;
    }

    private static Driver mInstance;
    private final ExpCurve translationStickCurve, rotationStickCurve;

    private Driver() {
        super(Settings.name, Settings.port);

        translationStickCurve = new ExpCurve(Settings.kTranslationExpVal, 0, 1, Settings.kDeadzone);
        rotationStickCurve = new ExpCurve(Settings.kRotationExpVal, 0, 1, Settings.kDeadzone);
    }

    public static Driver getInstance() {
        if (mInstance == null) {
            mInstance = new Driver();
        }
        return mInstance;
    }

    @Override
    public void setupTeleopButtons() {
        // Drivetrain Commands
        

        // controller.square().whileTrue(RobotCommands.primeSpeaker(SetAngleShooterPivot.Preset.kShooterNear));
        // controller.cross().whileTrue(RobotCommands.prime());

        // controller.R1().onTrue(RobotCommands.spitNote());

        // controller.L1().onTrue(RobotCommands.zero());

        //controller.povDown().whileTrue(DrivetrainCommands.driveAndLockTarget(this::getDriveTranslation));
        //controller.povDown().whileTrue(ShooterPivotCommands.tuneLockSpeaker(() -> Rotation2d.fromDegrees(45)));

        // controller.L1().whileTrue(new ConditionalCommand(
        //     DrivetrainCommands.turnToAngle(Rotation2d.fromDegrees(28.51)),
        //     DrivetrainCommands.turnToAngle(Rotation2d.fromDegrees(-28.51)),
        //     () -> {
        //         var alliance = DriverStation.getAlliance();
        //         if (alliance.isPresent()) {
        //             return alliance.get() == DriverStation.Alliance.Red;
        //         }

        //         return false;
        //     }
        // ));
    }

    @Override
    public void setupDisabledButtons() {}

    @Override
    public void setupTestButtons() {}

    public Translation2d getDriveTranslation() {
        final var xComponent = translationStickCurve.calculate(-controller.getLeftX());
        final var yComponent = translationStickCurve.calculate(-controller.getLeftY());
        // Components are reversed because field coordinates are opposite of joystick coordinates
        return new Translation2d(yComponent, xComponent);
    }

    public double getDriveRotation() {
        return rotationStickCurve.calculate(-controller.getRightX());
    }
}
