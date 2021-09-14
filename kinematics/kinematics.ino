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
    GOING_BY_POINTS,
    HEAD_MOVING
} work_state;

enum {
    UP,
    DOWN
} head_position;

const float SPEED_X = 400; // для одного движка, больше 540 уже клинит иногда
const float SPEED_Y = 400; // для одного движка, больше 540 уже клинит иногда
const float SPEED_Z = 300; // для двух движков в параллель, больше 450 уже клинит иногда

bool calibration_mode_is_setted = false;
bool work_mode_is_setted = false;

// массив точек:
// первый элемент - положение головки(0 - поднята, 1 - опущена)
// второй элемент - координата х
// третий элемент - координата у

// прямоугольник
const int POINTS[][3] = {
    {0, 3000, 1000},
    {1, 3000, 2000},
    {1, 4000, 2000},
    {1, 4000, 1000},
    {1, 2900, 1000}
};

//прямоугольный треугольник
/*int POINTS[][3] = {
    {0, 1700, 600},
    {1, 1700, 3600},
    {1, 4700, 600},
    {1, 1700, 600},
};*/

const int POINTS_SIZE = sizeof(POINTS) / sizeof(POINTS[0]);
int current_point = 0;

// активация режима выполнения каждого шага по нажатию кнопки
const bool NEXT_STEP_FROM_BUTTON = false;
bool await_button = true;

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
            stepper_x.setSpeed(SPEED_X);
        }
    } else if (current_axis == Y) {
        if (!stepper_y.tick()) {
            stepper_y.setSpeed(SPEED_Y);
        }
    } else if (current_axis == Z) {
        if (!stepper_z.tick()) {
            stepper_z.setSpeed(SPEED_Z);
        }
    }
}

void back() {
    if (current_axis == X) {
        if (!stepper_x.tick()) {
            stepper_x.setSpeed(-SPEED_X);
        }
    } else if (current_axis == Y) {
        if (!stepper_y.tick()) {
            stepper_y.setSpeed(-SPEED_Y);
        }
    } else if (current_axis == Z) {
        if (!stepper_z.tick()) {
            stepper_z.setSpeed(-SPEED_Z);
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

bool goToZero() {
    bool x_zero_reached = false;
    bool y_zero_reached = false;
    // bool z_zero_reached = false;
    bool zero_reached = false;

    // Если одновременно включить 4 движка, то питание проседает,
    // поэтому сначала выгоняю в 0 оси X, Y, а потом Z.
    // Да и поскольку сейчас 0 у Z это нижняя точка, то надо сначала увести стол.

    // если голова поднята, то выгоняем оси, иначе сначала поднимаем голову
    if (head_position == UP) {
        if (stepper_x.getCurrent() != 0) {
            if (!stepper_x.tick()) {
                stepper_x.setTarget(0);
            }
        } else {
            stepper_x.brake();
            x_zero_reached = true;
        }

        if (stepper_y.getCurrent() != 0) {
            if (!stepper_y.tick()) {
                stepper_y.setTarget(0);
            }
        } else {
            stepper_y.brake();
            y_zero_reached = true;
        }
    } else {
        moveHead(0);
    }

    if (x_zero_reached && y_zero_reached) {
        bool head_moved = moveHead(1);
        if (head_moved) {
            zero_reached = true;
        }
    }

    return zero_reached;
}

bool goToPoint(int x, int y) {
    bool x_reached = false;
    bool y_reached = false;
    bool point_reached = false;

    if (stepper_x.getCurrent() != x) {
        if (!stepper_x.tick()) {
            stepper_x.setTarget(x);
        }
    } else {
        stepper_x.brake();
        x_reached = true;
    }

    if (stepper_y.getCurrent() != y) {
        if (!stepper_y.tick()) {
            stepper_y.setTarget(y);
        }
    } else {
        stepper_y.brake();
        y_reached = true;
    }

    if (x_reached && y_reached) {
        point_reached = true;
    }

    return point_reached;
}

bool moveHead(int z) {
    long up_pos = -500;
    long down_pos = 0;
    bool head_moved = false;

    if (z == 0) {
        if (stepper_z.getCurrent() != up_pos) {
            if (!stepper_z.tick()) {
                stepper_z.setTarget(up_pos);
            }
        } else {
            stepper_z.brake();
            head_moved = true;
            head_position = UP;
        }
    } else if (z == 1) {
        if (stepper_z.getCurrent() != down_pos) {
            if (!stepper_z.tick()) {
                stepper_z.setTarget(down_pos);
            }
        } else {
            stepper_z.brake();
            head_moved = true;
            head_position = DOWN;
        }
    }

    return head_moved;
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

    stepper_x.setMaxSpeed(SPEED_X);
    stepper_y.setMaxSpeed(SPEED_Y);
    stepper_z.setMaxSpeed(SPEED_Z);

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

    work_state = NONE;
    head_position = DOWN;
    Serial.println("ready");
}

void loop()	{
    detect_current_mode();

    if (current_mode == CALIBRATION) {
        setupCalibrationMode();
        detect_current_axis();

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

        // кнопка 3 - отвод осей в 0
        if (digitalRead(bt3) == HIGH) {
            work_state = GOING_TO_ZERO;
        // кнопка 1 - пуск движения по массиву точек
        } else if (digitalRead(bt1) == HIGH) {
            // защита от дребезга контактов
            delay(10);
            if (digitalRead(bt1) == HIGH) {
                // ждем когда кнопка будет отжата
                while (true) {
                    if (digitalRead(bt1) == LOW) {
                        delay(10);
                        if (digitalRead(bt1) == LOW) {
                            work_state = GOING_BY_POINTS;
                            await_button = false;
                            break;
                        }
                    }
                }
            }
        // кнопка 2 - поднять/опустить голову (только если не в работе)
        } else if (digitalRead(bt2) == HIGH) {
            if (work_state == NONE) {
                delay(10);
                if (digitalRead(bt2) == HIGH) {
                    while (true) {
                        if (digitalRead(bt2) == LOW) {
                            delay(10);
                            if (digitalRead(bt2) == LOW) {
                                work_state = HEAD_MOVING;
                                break;
                            }
                        }
                    }
                }
            }
        }

        if (work_state == GOING_TO_ZERO) {
            bool zero_reached = goToZero();
            
            if (zero_reached) {
                work_state = NONE;
            }
        } else if (work_state == GOING_BY_POINTS) {
            // если не ждем нажатия кнопки, то работаем
            if (!await_button) {
                int z = POINTS[current_point][0];
                bool head_moved = moveHead(z);

                if (head_moved) {
                    int x = POINTS[current_point][1];
                    int y = POINTS[current_point][2];
                    bool point_reached = goToPoint(x, y);

                    if (point_reached) {
                        Serial.print(x);
                        Serial.print(" ");
                        Serial.print(y);
                        Serial.println(" point reached");

                        current_point++;
                        if (current_point == POINTS_SIZE) {
                            Serial.println("End of POINTS array");
                            Serial.println("Going to zero");
                            current_point = 0;
                            work_state = GOING_TO_ZERO;
                        }

                        // если включен режим "шаг по нажатию кнопки", то сбрасываем флаг ожидания
                        if (NEXT_STEP_FROM_BUTTON) {
                            await_button = true;
                        }
                    }
                }
            }
        } else if (work_state == HEAD_MOVING) {
            bool head_moved = false;

            if (head_position == DOWN) {
                head_moved = moveHead(0);
            } else if (head_position == UP) {
                head_moved = moveHead(1);
            }

            if (head_moved) {
                work_state = NONE;
            }
        }
    }

}