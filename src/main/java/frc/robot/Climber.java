package frc.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
// import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.Constants.*;

public class Climber {
    public WPI_TalonFX left;
    public WPI_TalonFX right;

    public Climber(){
        left = new WPI_TalonFX(Ports.left_climber_port);
        left.setInverted(true);
        right = new WPI_TalonFX(Ports.right_climber_port);
        right.setInverted(false);
        left.setNeutralMode(NeutralMode.Brake);
        right.setNeutralMode(NeutralMode.Brake);
        left.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        right.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        left.setSelectedSensorPosition(0);
        right.setSelectedSensorPosition(0);
    }

    public void set_speeds(double left_speed, double right_speed){
        left.set(left_speed*-1);
        right.set(right_speed);
    }

    public void set_speeds(double speed){
        left.set(speed*-1);
        right.set(speed);
    }

    public void set_voltages(double left_volt, double right_volt){
        left.setVoltage(left_volt);
        right.setVoltage(right_volt);
    }

    public double get_left_pos(){
        return Math.abs(left.getSelectedSensorPosition());
    }

    public double get_right_pos(){
        return Math.abs(right.getSelectedSensorPosition());
    }

    public void reset_pos(){
        left.setSelectedSensorPosition(0);
        right.setSelectedSensorPosition(0);
    }

    public void stop(){
        left.set(0);
        right.set(0);
    }

    public void stop(int side){
        if(side==0){
            left.set(0);
        }else if(side==1){
            right.set(0);
        }
    }
}
