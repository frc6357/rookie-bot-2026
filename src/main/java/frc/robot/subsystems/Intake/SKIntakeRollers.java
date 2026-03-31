package frc.robot.subsystems.Intake;

import static frc.robot.Ports.IntakePorts.kIntakeRollerMotor;
import static frc.robot.Konstants.IntakeConstants.kIntakeRollersP;
import static frc.robot.Konstants.IntakeConstants.kIntakeRollersRadius;
import static frc.robot.Konstants.IntakeConstants.kIntakeRollersI;
import static frc.robot.Konstants.IntakeConstants.kIntakeRollersD;
import static frc.robot.Konstants.IntakeConstants.kIntakeRollersF;
import static frc.robot.Konstants.IntakeConstants.kIntakeRollersS;
import static frc.robot.Konstants.IntakeConstants.kIntakeRollersTolerance;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SKIntakeRollers extends SubsystemBase {

    SparkFlex intakeRollerMotor = new SparkFlex(kIntakeRollerMotor.ID, MotorType.kBrushless);
    SparkClosedLoopController intakeRollerController = intakeRollerMotor.getClosedLoopController();

    double targetRPS;

    SKIntakeRollers() {
        
        SparkFlexConfig config = new SparkFlexConfig();
        config.idleMode(IdleMode.kCoast);
        config.closedLoop.pid(
            kIntakeRollersP,
            kIntakeRollersI,
            kIntakeRollersD
        );
        config.closedLoop.feedForward.kV(kIntakeRollersF);
        config.closedLoop.feedForward.kS(kIntakeRollersS);

        intakeRollerMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * Method to run the intake rollers at a specified speed (in RPS)
     * @param voltage The desired speed of the intake rollers in revolutions per second
     */
    public void runIntakeRollers(double velocity) {
        double rps = velocity/kIntakeRollersRadius;
        targetRPS = rps;
        intakeRollerController.setSetpoint(rps, ControlType.kVelocity);
    }

    /**
     * Method to stop the intake rollers by setting the motor speed to zero
     */
    public void stopIntakeRollers() {
        targetRPS = 0;
        intakeRollerController.setSetpoint(0, ControlType.kVelocity);
    }

    public boolean isRollersAtSpeed() {
        return Math.abs(intakeRollerMotor.getEncoder().getVelocity()-targetRPS) < kIntakeRollersTolerance;
    }

    /**
     * Command to run the intake rollers at a specified speed (in RPS) and stop them when the command ends
     * @param voltage The desired speed of the intake rollers in revolutions per second
     * @return A command that runs the intake rollers at the specified speed and stops them when the command ends
     */
    public Command runIntakeRollersCommand(double voltage) {
        return this.startEnd(
            () -> runIntakeRollers(voltage), 
            () -> stopIntakeRollers());
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        logOutputs();
    }

    private void logOutputs() {
        Logger.recordOutput("Intake Rollers Target RPS", targetRPS);
        Logger.recordOutput("Is Intake Rollers at Speed", isRollersAtSpeed());
        Logger.recordOutput("Intake Rollers RPS", intakeRollerMotor.getEncoder().getVelocity());
    }
    
}
