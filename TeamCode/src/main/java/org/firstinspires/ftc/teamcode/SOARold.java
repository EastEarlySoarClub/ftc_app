package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;


@TeleOp(name="SOAR old", group="Falcon")
@Disabled
public class SOARold extends OpMode {

    DcMotor motorRight;
    DcMotor motorLeft;
    DcMotor scoop;
    DcMotor shoot;
    // float positionLeft = 0;
    //float positionRight = 0;
    private ElapsedTime shootTime = new ElapsedTime();

    public void start() {

        motorRight = hardwareMap.dcMotor.get("motorRight");
        motorLeft = hardwareMap.dcMotor.get("motorLeft");
        motorLeft.setDirection(DcMotor.Direction.REVERSE);
        scoop = hardwareMap.dcMotor.get("scoop");
        shoot = hardwareMap.dcMotor.get("shoot");
        shoot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }

    @Override
    public void init() {


    }

    @Override
    public void loop() {


        float left = gamepad1.left_stick_y;
        float right = gamepad1.right_stick_y;


        right = Range.clip(right, -1, 1);
        left = Range.clip(left, -1, 1);

        right = (float) scaleInput(right);
        left = (float) scaleInput(left);

        //float left2 = -gamepad2.left_stick_y;
        float right2 = -gamepad2.right_stick_y;


        right2 = Range.clip(right2, -1, 1);
        //left2 = Range.clip(left2, -1, 1);

        right2 = (float) scaleInput(right2);
        //left2 = (float) scaleInput(left2);
        double shootInitial = shoot.getCurrentPosition();

        motorRight.setPower(right);
        motorLeft.setPower(left);
        shoot.setPower(right2);
        telemetry.addData("Shooter:","%d",
                shoot.getCurrentPosition());
        telemetry.addData("Motors:", "Left %d, Right%d",motorLeft.getCurrentPosition(),motorRight.getCurrentPosition());
        telemetry.update();



        if (gamepad2.right_bumper) {

            scoop.setPower(.4);

        }

        if (gamepad2.left_bumper) {
            scoop.setPower(-.4);

        }
        if (!gamepad2.left_bumper && !gamepad2.right_bumper) {
            scoop.setPower(0);

        }
        if (gamepad1.a) {
            motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            shoot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            shoot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        }

    }


    @Override
    public void stop(){


    }
    double scaleInput(double dVal)  {
        double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);
        if (index < 0) {
            index = -index;
        } else if (index > 16) {
            index = 16;
        }

        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        return dScale;
    }
}

