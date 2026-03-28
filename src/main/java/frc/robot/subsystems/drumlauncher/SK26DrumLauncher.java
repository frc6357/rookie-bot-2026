package frc.robot.subsystems.drumlauncher;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Konstants.DrumLauncherConstants;
import frc.robot.Ports.DrumLauncherPorts;

/**
 * Drum launcher subsystem for FRC Team 6357 "Spring Konstant" – 2026.
 *
 * <p>Static two-wheel launcher driven by two NEO Vortex motors on Spark Flex
 * controllers.  The bottom wheel primarily provides propulsion/range while the
 * top wheel adjusts trajectory by running faster or slower relative to the
 * bottom wheel.
 *
 * <p>Velocity closed-loop control is performed on-controller via
 * {@link SparkClosedLoopController}.  All tunable values (PID, gear ratio,
 * current limits, etc.) are placeholder constants at the top of this file and
 * marked with {@code TODO} where characterisation data is still needed.
 */
public class SK26DrumLauncher extends SubsystemBase {

    // ========================== Shot Mode Enum ==========================

    /** Operating modes for the drum launcher. */
    public enum ShotMode {
        /** Wheels stopped, no command. */
        IDLE,
        /** Scoring into the hub at a calculated velocity. */
        SCORE,
        /** Shuttle / lob shot at reduced velocity. */
        SHUTTLE,
        /** Operator-commanded RPM (bypass auto-calculations). */
        MANUAL,
        /** Hard stop – wheels coast/brake to zero. */
        STOPPED
    }

    // ========================== Hardware ==========================

    private final SparkFlex m_bottomMotor;
    private final SparkFlex m_topMotor;

    private final RelativeEncoder m_bottomEncoder;
    private final RelativeEncoder m_topEncoder;

    private final SparkClosedLoopController m_bottomController;
    private final SparkClosedLoopController m_topController;

    // ========================== Internal State ==========================

    private double targetBottomRPM = 0.0;
    private double targetTopRPM    = 0.0;
    private ShotMode shotMode      = ShotMode.IDLE;
    private double lastDistanceMeters = 0.0;

    // ========================== Constructor ==========================

    public SK26DrumLauncher() {
        // --- Instantiate motors ---
        m_bottomMotor = new SparkFlex(DrumLauncherPorts.kBottomMotor.ID, MotorType.kBrushless);
        m_topMotor    = new SparkFlex(DrumLauncherPorts.kTopMotor.ID,    MotorType.kBrushless);

        // --- Grab encoders and closed-loop controllers ---
        m_bottomEncoder    = m_bottomMotor.getEncoder();
        m_topEncoder       = m_topMotor.getEncoder();
        m_bottomController = m_bottomMotor.getClosedLoopController();
        m_topController    = m_topMotor.getClosedLoopController();

        // --- Configure bottom motor ---
        SparkFlexConfig bottomConfig = new SparkFlexConfig();
        bottomConfig.inverted(DrumLauncherConstants.kBottomMotorInverted);
        bottomConfig.idleMode(IdleMode.kCoast);
        bottomConfig.smartCurrentLimit(DrumLauncherConstants.kSmartCurrentLimitAmps);
        bottomConfig.encoder.velocityConversionFactor(1.0); // native RPM, no conversion
        bottomConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(DrumLauncherConstants.kBottomP)
            .i(DrumLauncherConstants.kBottomI)
            .d(DrumLauncherConstants.kBottomD);
        bottomConfig.closedLoop.feedForward
            .kV(DrumLauncherConstants.kBottomFF);

        m_bottomMotor.configure(
            bottomConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);

        // --- Configure top motor ---
        SparkFlexConfig topConfig = new SparkFlexConfig();
        topConfig.inverted(DrumLauncherConstants.kTopMotorInverted);
        topConfig.idleMode(IdleMode.kCoast);
        topConfig.smartCurrentLimit(DrumLauncherConstants.kSmartCurrentLimitAmps);
        topConfig.encoder.velocityConversionFactor(1.0);
        topConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(DrumLauncherConstants.kTopP)
            .i(DrumLauncherConstants.kTopI)
            .d(DrumLauncherConstants.kTopD);
        topConfig.closedLoop.feedForward
            .kV(DrumLauncherConstants.kTopFF);

        m_topMotor.configure(
            topConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);

        // Start in IDLE and set the default command
        setIdle();
        setDefaultCommand(idleCommand());
    }

    // ========================== Public API ==========================

    /**
     * Direct control path.  Sets both wheel target RPMs and commands the
     * Spark Flex closed-loop controllers at the requested velocities.
     *
     * <p>No gear-ratio or conversion logic is applied internally; the caller
     * is responsible for providing the desired <b>motor</b> RPM values.
     *
     * @param bottomRPM target RPM for the bottom motor
     * @param topRPM    target RPM for the top motor
     */
    public void setTargetRPMs(double bottomRPM, double topRPM) {
        targetBottomRPM = bottomRPM;
        targetTopRPM    = topRPM;

        m_bottomController.setSetpoint(targetBottomRPM, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
        m_topController.setSetpoint(targetTopRPM, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    }

    /**
     * Sets the launcher into IDLE mode – a slow spin to keep flywheels warm
     * and reduce spin-up time.
     */
    public void setIdle() {
        shotMode = ShotMode.IDLE;
        setTargetRPMs(DrumLauncherConstants.kIdleRPM, DrumLauncherConstants.kIdleRPM);
    }

    /**
     * Stops both launcher wheels (target 0 RPM).
     */
    public void stop() {
        shotMode = ShotMode.STOPPED;
        setTargetRPMs(0.0, 0.0);
    }

    /**
     * Placeholder – computes the bottom &amp; top RPM from a distance to the
     * target.  Currently stores the distance and sets a default target; real
     * shot-math will be added later.
     *
     * @param distanceMeters distance from the robot to the scoring target
     * @param mode           the {@link ShotMode} to set (e.g. SCORE or SHUTTLE)
     */
    public void runAtDistance(double distanceMeters, ShotMode mode) {
        lastDistanceMeters = distanceMeters;
        shotMode = mode;

        // TODO: Replace with real distance-to-RPM lookup / polynomial.
        //       This placeholder spins both wheels at a fixed speed.
        double bottomRPM = 3000.0; // TODO: calculate from distanceMeters
        double topRPM    = 3000.0; // TODO: calculate from distanceMeters
        setTargetRPMs(bottomRPM, topRPM);
    }

    // ========================== Command Factories ==========================

    /**
     * Creates a command that holds the launcher in IDLE mode (slow spin to
     * keep flywheels warm).  Runs forever until interrupted.
     *
     * @return the idle command
     */
    public Command idleCommand() {
        return run(() -> setIdle()).withName("LauncherIdle");
    }

    /**
     * Creates a command that stops both launcher wheels.  Runs forever until
     * interrupted.
     *
     * @return the stop command
     */
    public Command stopCommand() {
        return run(() -> stop()).withName("LauncherStop");
    }

    /**
     * Creates a command that sets the launcher to the given mode at the RPM
     * calculated from the supplied distance.  The distance supplier is polled
     * every cycle so the shot can track a moving target.  Runs forever until
     * interrupted.
     *
     * @param distanceMetersSupplier supplier providing the current distance to
     *                               the scoring target, in metres
     * @param mode                   the {@link ShotMode} to set (e.g. SCORE or SHUTTLE)
     * @return the run-at-distance command
     */
    public Command runAtDistanceCommand(Supplier<Double> distanceMetersSupplier, ShotMode mode) {
        return run(() -> runAtDistance(distanceMetersSupplier.get(), mode))
            .withName("LauncherRunAtDistance");
    }

    // ========================== Getters ==========================

    /** @return actual bottom wheel velocity in RPM */
    @AutoLogOutput(key = "Launcher/BottomRPM")
    public double getBottomRPM() {
        return m_bottomEncoder.getVelocity();
    }

    /** @return actual top wheel velocity in RPM */
    @AutoLogOutput(key = "Launcher/TopRPM")
    public double getTopRPM() {
        return m_topEncoder.getVelocity();
    }

    /** @return commanded bottom wheel velocity in RPM */
    @AutoLogOutput(key = "Launcher/BottomTargetRPM")
    public double getBottomTargetRPM() {
        return targetBottomRPM;
    }

    /** @return commanded top wheel velocity in RPM */
    @AutoLogOutput(key = "Launcher/TopTargetRPM")
    public double getTopTargetRPM() {
        return targetTopRPM;
    }

    /** @return the current {@link ShotMode} */
    @AutoLogOutput(key = "Launcher/ShotMode")
    public String getShotModeString() {
        return shotMode.name();
    }

    /** @return the current {@link ShotMode} enum value */
    public ShotMode getShotMode() {
        return shotMode;
    }

    /**
     * @return {@code true} if both wheels are within {@link #kRPMTolerance} of
     *         their target velocities
     */
    @AutoLogOutput(key = "Launcher/AtTarget")
    public boolean atTarget() {
        return Math.abs(getBottomRPM() - targetBottomRPM) <= DrumLauncherConstants.kRPMTolerance
            && Math.abs(getTopRPM()    - targetTopRPM)    <= DrumLauncherConstants.kRPMTolerance;
    }

    /**
     * Indicates that the launcher is ready for a game piece to be fed.
     *
     * <p>Currently returns the same value as {@link #atTarget()}.  This may
     * later include additional conditions (e.g.&nbsp;indexer state, minimum
     * RPM thresholds).
     *
     * @return {@code true} when it is safe to feed a game piece
     */
    @AutoLogOutput(key = "Launcher/ReadyToFeed")
    public boolean readyToFeed() {
        return atTarget();
    }

    // ========================== Periodic ==========================

    @Override
    public void periodic() {
        // AdvantageKit @AutoLogOutput handles the annotated getters above.
        // Additional Logger.recordOutput calls below for error & distance data.

        Logger.recordOutput("Launcher/BottomError", targetBottomRPM - getBottomRPM());
        Logger.recordOutput("Launcher/TopError",    targetTopRPM    - getTopRPM());
        Logger.recordOutput("Launcher/LastDistanceMeters", lastDistanceMeters);
    }
}
