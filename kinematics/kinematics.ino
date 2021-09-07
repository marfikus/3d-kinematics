#include <GyverStepper.h>

GStepper<STEPPER4WIRE> stepper_x(2048, 13, 11, 12, 10);
GStepper<STEPPER4WIRE> stepper_y(2048, 9, 7, 8, 6);
GStepper<STEPPER4WIRE> stepper_z(2048, 5, 3, 4, 2);

byte bt1 = A0;
byte bt2 = A1;
byte sw1_1 = A2;
byte sw1_2 = A3;
byte sw2 = A4;
byte bt3 = A5;

enum {
    X,
    Y,
    Z
} current_axis;

enum {
    CALIBRATION,
    WORK
} current_mode;

enum {
    NONE,
    GOING_TO_ZERO,
    GOING_BY_POINTS
} work_state;

float speed_x = 400; // для одного движка, больше 540 уже клинит иногда
float speed_y = 400; // для одного движка, больше 540 уже клинит иногда
float speed_z = 300; // для двух движков в параллель, больше 450 уже клинит иногда

bool calibration_mode_is_setted = false;
bool work_mode_is_setted = false;

int points[][2] = {
    {1000, 1000},
    {1000, 2000},
    {2000, 2000},
    {2000, 1000},
    {1000, 1000},
};
int points_size = 5;

int point_counter = 0;
bool point_reached = false;
bool zero_reached = false;
bool await_start_from_button = true;

void detect_current_mode() {
    if (digitalRead(sw2) == HIGH) {
        current_mode = CALIBRATION;
    } else {
        current_mode = WORK;
    }
}

void detect_current_axis() {
    if (digitalRead(sw1_1) == HIGH) {
        current_axis = X;
    } else if (digitalRead(sw1_2) == HIGH) {
        current_axis = Y;
    } else {
        current_axis = Z;
    }
}

void forward() {
    if (current_axis == X) {
        if (!stepper_x.tick()) {
            stepper_x.setSpeed(speed_x);
        }
    } else if (current_axis == Y) {
        if (!stepper_y.tick()) {
            stepper_y.setSpeed(speed_y);
        }
    } else if (current_axis == Z) {
        if (!stepper_z.tick()) {
            stepper_z.setSpeed(speed_z);
        }
    }
}

void back() {
    if (current_axis == X) {
        if (!stepper_x.tick()) {
            stepper_x.setSpeed(-speed_x);
        }
    } else if (current_axis == Y) {
        if (!stepper_y.tick()) {
            stepper_y.setSpeed(-speed_y);
        }
    } else if (current_axis == Z) {
        if (!stepper_z.tick()) {
            stepper_z.setSpeed(-speed_z);
        }
    }
}

void stop() {
    if (current_axis == X) {
        stepper_x.brake();
    } else if (current_axis == Y) {
        stepper_y.brake();
    } else if (current_axis == Z) {
        stepper_z.brake();
    }
}

void setZero() {
    if (current_axis == X) {
        stepper_x.reset();
    } else if (current_axis == Y) {
        stepper_y.reset();
    } else if (current_axis == Z) {
        stepper_z.reset();
    }
}

void goToZero() {
    bool x_zero_reached = false;
    bool y_zero_reached = false;
    bool z_zero_reached = false;

    // if (current_axis == X) {
        if (stepper_x.getCurrent() != 0) {
            if (!stepper_x.tick()) {
                stepper_x.setTarget(0);
            }
        } else {
            // work_state = NONE;
            x_zero_reached = true;
        }
    // } else if (current_axis == Y) {
        if (stepper_y.getCurrent() != 0) {
            if (!stepper_y.tick()) {
                stepper_y.setTarget(0);
            }
        } else {
            // work_state = NONE;
            y_zero_reached = true;
        }
    // } else if (current_axis == Z) {
        if (stepper_z.getCurrent() != 0) {
            if (!stepper_z.tick()) {
                stepper_z.setTarget(0);
            }
        } else {
            // work_state = NONE;
            z_zero_reached = true;
        }
    // }   

    if (x_zero_reached && y_zero_reached && z_zero_reached) {
        zero_reached = true;
    }        
}

void goToPoint(int x, int y) {
    bool x_reached = false;
    bool y_reached = false;

    if (stepper_x.getCurrent() != x) {
        if (!stepper_x.tick()) {
            stepper_x.setTarget(x);
        }
    } else {
        x_reached = true;

        if (stepper_y.getCurrent() != y) {
            if (!stepper_y.tick()) {
                stepper_y.setTarget(y);
            }
        } else {
            y_reached = true;
        }        
    }

    if (x_reached && y_reached) {
        point_reached = true;
    }
}

void setupCalibrationMode() {
    if (calibration_mode_is_setted) {
        return;
    }

    stepper_x.setRunMode(KEEP_SPEED);
    stepper_y.setRunMode(KEEP_SPEED);
    stepper_z.setRunMode(KEEP_SPEED);

    stepper_x.setAcceleration(0);
    stepper_y.setAcceleration(0);
    stepper_z.setAcceleration(0);

    stepper_x.autoPower(true);
    stepper_y.autoPower(true);
    stepper_z.autoPower(true);

    calibration_mode_is_setted = true;
    work_mode_is_setted = false;
}

void setupWorkMode() {
    if (work_mode_is_setted) {
        return;
    }

    stepper_x.setRunMode(FOLLOW_POS);
    stepper_y.setRunMode(FOLLOW_POS);
    stepper_z.setRunMode(FOLLOW_POS);

    stepper_x.setMaxSpeed(speed_x);
    stepper_y.setMaxSpeed(speed_y);
    stepper_z.setMaxSpeed(speed_z);

    stepper_x.setAcceleration(0);
    stepper_y.setAcceleration(0);
    stepper_z.setAcceleration(0);

    stepper_x.autoPower(true);
    stepper_y.autoPower(true);
    stepper_z.autoPower(true);        

    work_mode_is_setted = true;
    calibration_mode_is_setted = false;
}

void setup() {
    // setupWorkMode();
    Serial.begin(9600);
	
	pinMode(bt1, INPUT);
	pinMode(bt2, INPUT);
    pinMode(sw1_1, INPUT);
    pinMode(sw1_2, INPUT);
    pinMode(sw2, INPUT);
    pinMode(bt3, INPUT);
}

void loop()	{
    detect_current_mode();
    detect_current_axis();

    if (current_mode == CALIBRATION) {
        setupCalibrationMode();

    	if (digitalRead(bt1) == HIGH) {
            forward();
    	} else if (digitalRead(bt2) == HIGH) {
            back();
    	} else {
            stop();
    	}

        if (digitalRead(bt3) == HIGH) {
            setZero();
        }

    } else if (current_mode == WORK) {
        setupWorkMode();

        if (digitalRead(bt3) == HIGH) {
            work_state = GOING_TO_ZERO;
        } else if (digitalRead(bt1) == HIGH) {
            // защита от дребезга контактов
            delay(10);
            if (digitalRead(bt1) == HIGH) {
                while (true) {
                    if (digitalRead(bt1) == LOW) {
                        delay(10);
                        if (digitalRead(bt1) == LOW) {
                            work_state = GOING_BY_POINTS;
                            await_start_from_button = false;
                            break;
                        }
                    }
                }
            }
        }

        if (work_state == GOING_TO_ZERO) {
            goToZero();

            if (zero_reached) {
                work_state = NONE;
                zero_reached = false;
            }
        } else if (work_state == GOING_BY_POINTS) {

            // if (!await_start_from_button) {

                int x = points[point_counter][0];
                int y = points[point_counter][1];
                goToPoint(x, y);

                if (point_reached) {
                    // delay(1000);
                    Serial.print(x);
                    Serial.print(" ");
                    Serial.print(y);
                    Serial.println(" point_reached");
                    if (point_counter < points_size - 1) {
                        point_counter++;
                    } else {
                        Serial.println("End of points array");
                        point_counter = 0;
                        work_state = GOING_TO_ZERO;
                    }
                    point_reached = false;
                    await_start_from_button = true;
                }                
            // }


        }
    }

}