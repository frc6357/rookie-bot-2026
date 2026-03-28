package frc.robot.subsystems.Intake;

import static frc.robot.Ports.IntakePorts.kIntakeRollerMotor;

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

    double targetVoltage;

    SKIntakeRollers() {
        
        SparkFlexConfig config = new SparkFlexConfig();
        config.idleMode(IdleMode.kCoast);
        intakeRollerMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * Method to run the intake rollers at a specified speed (in RPS)
     * @param voltage The desired speed of the intake rollers in revolutions per second
     */
    public void runIntakeRollers(double voltage) {
        targetVoltage = voltage;
        intakeRollerController.setSetpoint(voltage, ControlType.kVoltage);
    }

    /**
     * Method to stop the intake rollers by setting the motor speed to zero
     */
    public void stopIntakeRollers() {
        targetVoltage = 0;
        intakeRollerController.setSetpoint(0, ControlType.kVoltage);
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
        Logger.recordOutput("Intake Roller Target Voltage", targetVoltage);
    }
    
}
