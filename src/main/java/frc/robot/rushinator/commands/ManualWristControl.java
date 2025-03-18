package frc.robot.rushinator.commands;

import frc.robot.rushinator.RushinatorWrist;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.DoubleSupplier;

public class ManualWristControl extends Command {
    private final DoubleSupplier joystickInput;
    private double manualTarget; // target angle in radians

    public ManualWristControl(DoubleSupplier joystickInput) {
        this.joystickInput = joystickInput;
        addRequirements(RushinatorWrist.getInstance());
    }

    @Override
    public void initialize() {
        manualTarget = RushinatorWrist.getInstance().getCurrentPos().getRadians();
        RushinatorWrist.getInstance().setTargetPosition(new Rotation2d(manualTarget));
    }

    @Override
    public void execute() {
        double scale = 0.05;
        double input = joystickInput.getAsDouble();

        manualTarget += input * scale;

        manualTarget = Math.max(0, Math.min(Math.PI, manualTarget));

        RushinatorWrist.getInstance().setTargetPosition(new Rotation2d(manualTarget));

        double currentAngleDegrees = Math.toDegrees(RushinatorWrist.getInstance().getCurrentPos().getRadians());
        double manualTargetDegrees = Math.toDegrees(manualTarget);
        System.out.println("Current Wrist Angle: " + currentAngleDegrees 
            + "°, Manual Target: " + manualTargetDegrees + "°");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
