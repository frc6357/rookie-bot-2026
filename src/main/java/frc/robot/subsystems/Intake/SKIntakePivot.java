package frc.robot.subsystems.Intake;

import static frc.robot.Ports.IntakePorts.kIntakePivotMotor;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SKIntakePivot extends SubsystemBase {

    SparkFlex intakePivotMotor = new SparkFlex(kIntakePivotMotor.ID, MotorType.kBrushless);

    SKIntakePivot() {
        
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
    
}
