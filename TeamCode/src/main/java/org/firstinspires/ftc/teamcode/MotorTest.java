package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class MotorTest extends OpMode {
    DcMotor flywheel;
    double desiredSpeed = 0;
    PIDController control = new PIDController(0.05,0,0);

    @Override
    public void init(){
        flywheel = hardwareMap.get(DcMotor.class, "flywheel");
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void loop(){
        if (gamepad1.right_bumper){
            desiredSpeed = desiredSpeed + 0.1;
            double command = control.update(desiredSpeed,
                    flywheel.getPower());
            // assign motor the PID output
            flywheel.setPower(command);
        }

        if (gamepad1.left_bumper){
            desiredSpeed = desiredSpeed - 0.1;
            double command = control.update(desiredSpeed,
                    flywheel.getPower());
            // assign motor the PID output
            flywheel.setPower(command);
        }
    }

    public void increaseSpeed(){
        desiredSpeed = desiredSpeed + 0.1;
        flywheel.setPower(desiredSpeed);
    }

    public void decreaseSpeed(){
        desiredSpeed = desiredSpeed - 0.1;
        flywheel.setPower(desiredSpeed);
    }

    public void maintainSpeed(){
        flywheel.setPower(desiredSpeed);
    }
}
