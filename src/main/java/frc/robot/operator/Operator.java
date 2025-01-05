package frc.robot.operator;

import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.crevolib.util.ExpCurve;
import frc.crevolib.util.Gamepad;
import frc.robot.Robot;

public class Operator extends Gamepad {
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
    private static Operator mInstance;

    ExpCurve shooterManualCurve;

    private Operator() {
        super(Settings.name, Settings.port);

       stickCurve = new ExpCurve(1, 0, 1, Settings.kDeadzone);
        
    }

    public static Operator getInstance() {
        if (mInstance == null) {
            mInstance = new Operator();
        }
        return mInstance;
    }

    @Override
    public void setupTeleopButtons() {
        
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

    public double getDriveRotation() {
        return -stickCurve.calculate(controller.getRightX());
    }
}
