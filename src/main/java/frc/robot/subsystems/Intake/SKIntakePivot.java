package frc.robot.subsystems.Intake;

import static frc.robot.Ports.IntakePorts.kIntakePivotMotor;

import org.littletonrobotics.junction.Logger;

import static frc.robot.Konstants.IntakeConstants.kIntakePivotP;
import static frc.robot.Konstants.IntakeConstants.kIntakePivotVoltage;
import static frc.robot.Konstants.IntakeConstants.kIntakePivotI;
import static frc.robot.Konstants.IntakeConstants.kIntakePivotD;
import static frc.robot.Konstants.IntakeConstants.kIntakePivotF;
import static frc.robot.Konstants.IntakeConstants.kIntakePivotS;
import static frc.robot.Konstants.IntakeConstants.kIntakePivotGearRatio;
import static frc.robot.Konstants.IntakeConstants.kIntakePivotUpAngle;
import static frc.robot.Konstants.IntakeConstants.kIntakePivotDownAngle;
import static frc.robot.Konstants.IntakeConstants.kIntakePivotAngleTolerance;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;
import lombok.Setter;

public class SKIntakePivot extends SubsystemBase {

    SparkFlex intakePivotMotor = new SparkFlex(kIntakePivotMotor.ID, MotorType.kBrushless);
    SparkClosedLoopController controller = intakePivotMotor.getClosedLoopController();
    RelativeEncoder encoder = intakePivotMotor.getEncoder();

    @Getter
    private double targetVoltage;

    @Getter
    private double targetIntakePivotAngle = kIntakePivotUpAngle;

    @Getter
    private boolean isDeployed = false;

    @Getter
    @Setter
    private double intakePivotOffset = 0.0;

    SKIntakePivot() {

        SparkFlexConfig config = new SparkFlexConfig();
        config.closedLoop.p(kIntakePivotP);
        config.closedLoop.i(kIntakePivotI);
        config.closedLoop.d(kIntakePivotD);
        config.closedLoop.feedForward
        .kV(kIntakePivotF)
        .kS(kIntakePivotS);
        intakePivotMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void deployIntake() {
        controller.setSetpoint(kIntakePivotVoltage, ControlType.kVoltage);
        targetIntakePivotAngle = kIntakePivotDownAngle;
    }

    public void pullIntakePivot() {
        controller.setSetpoint(-kIntakePivotVoltage, ControlType.kVoltage);
        targetIntakePivotAngle = kIntakePivotUpAngle;
    }

    public double getIntakePivotAngleDeg() {
        double revs = encoder.getPosition();
        double deg = revs * 360.0;
        deg /= kIntakePivotGearRatio;
        deg -= intakePivotOffset;
        return deg;
    }

    public double getIntakePivotAngleDegWrapped() {
        double deg = getIntakePivotAngleDeg();
        deg = ((deg + 180.0) % 360.0);
        if (deg < 0) deg += 360.0;
        return deg - 180.0;
    }

    public boolean isAngleInTolerance() {
        return Math.abs(getIntakePivotAngleDegWrapped() - targetIntakePivotAngle) < kIntakePivotAngleTolerance;
    }

    public Command deployIntakeCommand() {
        if(!isDeployed) {
            return this.startEnd(
                () -> deployIntake(), 
                () -> controller.setSetpoint(0, ControlType.kVoltage)).until(() -> isAngleInTolerance());
        } else {
            return null;
        }
    }

    public Command pullIntakePivotCommand() {
        if(isDeployed) {
            return this.startEnd(
                () -> pullIntakePivot(), 
                () -> controller.setSetpoint(0, ControlType.kVoltage)).until(() -> isAngleInTolerance());
        } else {
            return null;
        }
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        logOutputs();
    }

    private void logOutputs() {
        Logger.recordOutput("Is Deployed", isDeployed);
        Logger.recordOutput("Is Angle in Tolerance", isAngleInTolerance());
    }
    
}
