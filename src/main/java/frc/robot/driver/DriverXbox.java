package frc.robot.driver;

import java.util.Optional;

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
        // Drivetrain Commands



        // Future notice: A = speaker and x = amp wil be amp and speaker shoot on move
        // controller.a().whileTrue(DrivetrainCommands.shootSpeakerMode(() -> mInstance.getDriveTranslation(), () -> mInstance.getDriveRotation(), true, false));
        // controller.a().whileFalse(DrivetrainCommands.shootSpeakerMode(() -> mInstance.getDriveTranslation(), () -> mInstance.getDriveRotation(), false, false));

        // controller.x().whileFalse(DrivetrainCommands.shootAmpMode(() -> mInstance.getDriveTranslation(), () -> mInstance.getDriveRotation(), false, true));
        // controller.x().whileFalse(DrivetrainCommands.shootAmpMode(() -> mInstance.getDriveTranslation(), () -> mInstance.getDriveRotation(), false, false));



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
        // System.out.println("Colin Is GAY!!! " + new Translation2d(yComponent, xComponent).toString());
        return new Translation2d(yComponent, xComponent);
    }

    public void setDriveRotation(double requestedAngularVel) {
        reqAngularVel = requestedAngularVel;
    }

    public double getDriveRotation() {
        return rotationStickCurve.calculate(-controller.getRightX());
        // return reqAngularVel;
        // System.out.println("Is it true??? " + autoAim);
        // if (true) {
        //     final Drivetrain drivetrain = Drivetrain.getInstance();

        //     PIDController pidController = new PIDController(1.0, 0.0, 0.1);
            
        //     Rotation2d deltaTheta;
        //     Rotation2d targetAngle;
        //     Rotation2d kMaxAngularVelocity = Rotation2d.fromDegrees(1.0);

        //     Pose2d goalPose = null;
            

        //     final var mPoseEstimator = Vision.PoseEstimator.getInstance();
        //     final var robotPose = mPoseEstimator.getCurrentPose();

        //     Optional<Alliance> ally = DriverStation.getAlliance();
        //     if (ally.isPresent()) {
        //         if (ally.get() == Alliance.Blue) {
        //             goalPose = new Pose2d(new Translation2d(Units.inchesToMeters(-1.5), Units.inchesToMeters(218.42)), new Rotation2d(0));
        //         }
        //         if (ally.get() == Alliance.Red) {
        //             goalPose = new Pose2d(new Translation2d(Units.inchesToMeters(652.73), Units.inchesToMeters(218.42)), new Rotation2d(Units.degreesToRadians(180)));
        //         }
        //     }
        //     final var startingAngle = robotPose.getRotation();
        //     final var endAngle = goalPose.getTranslation().minus(robotPose.getTranslation()).getAngle().plus(Rotation2d.fromDegrees(180.0));
        //     deltaTheta = endAngle.minus(startingAngle);

        //     targetAngle = Rotation2d.fromDegrees(drivetrain.getGyroYaw().getDegrees() + deltaTheta.getDegrees());

        //     final var currentAngle = drivetrain.getGyroYaw();
        //     final var requestedAngularVelocity = Rotation2d.fromDegrees(MathUtil.clamp(
        //         pidController.calculate(currentAngle.getDegrees(), targetAngle.getDegrees()),
        //         -kMaxAngularVelocity.getDegrees(),
        //         kMaxAngularVelocity.getDegrees()
        //     ));

        //     System.out.println("requested Angular Vel: " + requestedAngularVelocity.getDegrees());
        //     return requestedAngularVelocity.getDegrees();
        // } else {
        //     return rotationStickCurve.calculate(-controller.getRightX());
        // }
    }
}