package frc.robot.climber;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList;
import java.util.function.Supplier;
import java.util.stream.Collectors;

public class Climber extends SubsystemBase {
    public static class Settings {
        static final int kClimberPivotId = 27;

        public static final Rotation2d kMaxPos = Rotation2d.fromRotations(270.0);
        public static final Rotation2d kMinPos = Rotation2d.fromRotations(0.0);

        public static final Rotation2d kHoldPos = Rotation2d.fromRotations(263.0);
        public static final Rotation2d kDeployPos = Rotation2d.fromRotations(180.0);

        static final double kMaxVoltage = 12.0f;
        static final int kMaxCurrent = 40;
    }

    private enum State {
        kFloating,
        kStowed,
        kDeploying,
        kDeployed,
        kRetracting,
        kHold;
    }

    public enum OperatingMode {
        kManual,
        kCompetition
    }

    private final SparkMax spark;
    private final RelativeEncoder encoder;

    private final Supplier<Boolean> deployProvider, retractProvider;
    private final Supplier<Double> overrideProvider;

    private State state;
    private final OperatingMode operatingMode;

    public Climber(Supplier<Boolean> deployProvider, Supplier<Boolean> retractProvider, Supplier<Double> overrideProvider) {
        this(deployProvider, retractProvider, overrideProvider, OperatingMode.kCompetition);
    }

    public Climber(Supplier<Boolean> deployProvider, Supplier<Boolean> retractProvider, Supplier<Double> overrideProvider, OperatingMode operatingMode) {
        this.deployProvider = deployProvider;
        this.retractProvider = retractProvider;
        this.overrideProvider = overrideProvider;

        this.spark = new SparkMax(Settings.kClimberPivotId, MotorType.kBrushless);
        this.encoder = spark.getEncoder();

        final var sparkConfig = new SparkMaxConfig()
                .inverted(true)
                .smartCurrentLimit(Settings.kMaxCurrent)
                .idleMode(IdleMode.kBrake) ;
        spark.configure(sparkConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        this.state = switch (operatingMode) {
            case kManual -> State.kFloating;
            case kCompetition -> State.kStowed;
        };
        this.operatingMode = operatingMode;
    }

    public void setVoltage(double voltage) {
        // Ensure we don't drive past either of the soft limits for the climber by clamping the voltage
        if ((voltage < 0 && getPos().getRotations() <= Settings.kMinPos.getRotations())
                || (voltage > 0 && getPos().getRotations() >= Settings.kMaxPos.getRotations())) {
            voltage = 0;
        }
        spark.setVoltage(voltage);
    }

    public Rotation2d getPos() {
        return Rotation2d.fromRotations(encoder.getPosition());
    }

    private State nextState(State state) {
        if (operatingMode == OperatingMode.kManual) {
            return State.kFloating;
        }

        if (overrideProvider.get() != 0.0f) {
            return State.kFloating;
        }

        ArrayList<State> observedStates = new ArrayList<>();
        while (true) {
            var previousState = state;
            observedStates.add(previousState);
            switch (state) {
                case kFloating:
                    if (deployProvider.get()) {
                        state = State.kDeploying;
                    } else if (retractProvider.get()) {
                        state = State.kRetracting;
                    }
                    break;
                case kStowed:
                    if (deployProvider.get()) {
                        state = State.kDeploying;
                    }
                    break;
                case kDeploying:
                    if (retractProvider.get()) {
                        state = State.kFloating;
                    } else if (getPos().getRotations() >= Settings.kDeployPos.getRotations()) {
                        state = State.kDeployed;
                    }
                    break;
                case kDeployed:
                    if (retractProvider.get()) {
                        state = State.kRetracting;
                    }
                    break;
                case kRetracting:
                    if (deployProvider.get()) {
                        state = State.kFloating;
                    } else if (getPos().getRotations() <= Settings.kHoldPos.getRotations()) {
                        state = State.kHold;
                    }
                    break;
                case kHold:
                    if (deployProvider.get()) {
                        state = State.kFloating;
                    }
                    // TODO: If a low enough current is detected for a sufficiently long time, we should transition to floating.
                    // This will prevent over-driving the climber when it isn't under load.
                    break;
            }

            // If we don't transition state in an iteration, we've reached the terminal state and can return
            if (previousState == state) {
                return state;
            }

            // If the list of observed states contains the current state, we've entered a loop. This shouldn't occur and
            // indicates an issue in the state machine, but we should log a warning before breaking the loop.
            if (observedStates.contains(state)) {
                System.out.printf("[WARN:CLIMBER] Encountered loop while transitioning climber state (states: %s)", observedStates.stream().map(Object::toString).collect(Collectors.joining(", ")));
                return state;
            }
        }
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Climber Angle (Rotations)", getPos().getRotations());
        SmartDashboard.putNumber("Climber Current (Amps)", spark.getOutputCurrent());

        switch (state) {
            case kFloating:
                setVoltage(overrideProvider.get() * Settings.kMaxVoltage);
                break;
            case kStowed:
                setVoltage(0);
                break;
            case kDeploying:
                setVoltage(8.0f);
                break;
            case kDeployed:
                setVoltage(0.0f);
                break;
            case kRetracting:
                setVoltage(-6.0f);
                break;
            case kHold:
                // TODO: Make this value proportional to the distance from the zero point
                setVoltage(-2.0f);
                break;
        }

        final var ns = nextState(state);
        if (ns != state) {
            System.out.printf("[INFO:CLIMBER] Transitioning state (previous: %s, new: %s)", state.toString(), ns.toString());
            state = ns;
        }
    }

    public static class DefaultCommand extends Command {
        public DefaultCommand(Climber climber) {
            addRequirements(climber);
        }
    }
}
