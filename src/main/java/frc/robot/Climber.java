package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.Constants.*;

public class Climber {
    public WPI_TalonSRX left;
    public WPI_TalonSRX right;

    public Climber(){
        left = new WPI_TalonSRX(Ports.left_climber_port);
        right = new WPI_TalonSRX(Ports.right_climber_port);
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
