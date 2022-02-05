package frc.robot;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.*;

public class DriveTrain {
    private MotorControllerGroup left;
    private MotorControllerGroup right;
    private DifferentialDrive dt;
    public DriveTrain(WPI_TalonFX front_left, WPI_TalonFX back_left, WPI_TalonFX front_right, WPI_TalonFX back_right){
        left = new MotorControllerGroup(front_left, back_left);
        right = new MotorControllerGroup(front_right, back_right);
        dt = new DifferentialDrive(left, right);

        left.setInverted(true);
        right.setInverted(false);
    }

    public void set_speeds(double xSpeed, double zRotation){
        dt.arcadeDrive(xSpeed, zRotation);
    }
}
