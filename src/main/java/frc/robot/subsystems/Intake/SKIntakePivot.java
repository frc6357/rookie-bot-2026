package frc.robot.subsystems.Intake;

import static frc.robot.Ports.IntakePorts.kIntakePivotMotor;
import static frc.robot.Konstants.IntakeConstants.kIntakePivotP;
import static frc.robot.Konstants.IntakeConstants.kIntakePivotPushVoltage;
import static frc.robot.Konstants.IntakeConstants.kIntakePivotI;
import static frc.robot.Konstants.IntakeConstants.kIntakePivotD;
import static frc.robot.Konstants.IntakeConstants.kIntakePivotF;
import static frc.robot.Konstants.IntakeConstants.kIntakePivotS;
import static frc.robot.Konstants.IntakeConstants.kIntakePivotOffset;
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

public class SKIntakePivot extends SubsystemBase {

    SparkFlex intakePivotMotor = new SparkFlex(kIntakePivotMotor.ID, MotorType.kBrushless);
    SparkClosedLoopController controller = intakePivotMotor.getClosedLoopController();
    RelativeEncoder encoder = intakePivotMotor.getEncoder();

    @Getter
    double targetVoltage;

    @Getter
    double targetIntakePivotAngle = kIntakePivotUpAngle;

    @Getter
    boolean isDeployed = false;

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
        controller.setSetpoint(kIntakePivotPushVoltage, ControlType.kVoltage);
        targetIntakePivotAngle = kIntakePivotDownAngle;
    }

    public double getIntakePivotAngle() {
        double revs = encoder.getPosition();
        double deg = revs * 360.0;
        deg -= kIntakePivotOffset;
        return deg;
    }

    public double getIntakePivotAngleWrapped() {
        double deg = getIntakePivotAngle();
        deg = ((deg + 180.0) % 360.0);
        if (deg < 0) deg += 360.0;
        return deg - 180.0;
    }

    public boolean isAngleInTolerance() {
        return Math.abs(getIntakePivotAngleWrapped() - targetIntakePivotAngle) < kIntakePivotAngleTolerance;
    }

    public Command deployIntakeCommand() {
        if(!isDeployed) {
            return this.startEnd(
                () -> deployIntake(), 
                () -> controller.setSetpoint(0, ControlType.kVoltage)).withTimeout(.5);
        } else {
            return this.startEnd(null, null);
        }
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
    
}
