package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "RingShotTest2 (Blocks to Java)", group = "")
public class RingShotTest2 extends LinearOpMode {

  private DcMotor motor4;
  private DcMotor motor5;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    motor4 = hardwareMap.dcMotor.get("motor4");
    motor5 = hardwareMap.dcMotor.get("motor5");

    // Put initialization blocks here.
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      while (opModeIsActive()) {
        // Put loop blocks here.
        motor4.setPower(2);
        motor5.setDirection(DcMotorSimple.Direction.REVERSE);
        motor5.setPower(2);
        telemetry.update();
      }
    }
  }
}
