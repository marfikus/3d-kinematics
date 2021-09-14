#include <GyverStepper.h>

GStepper<STEPPER4WIRE> stepperX(2048, 13, 11, 12, 10);
GStepper<STEPPER4WIRE> stepperY(2048, 9, 7, 8, 6);
GStepper<STEPPER4WIRE> stepperZ(2048, 5, 3, 4, 2);

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
} currentAxis;

enum {
    CALIBRATION,
    WORK
} currentMode;

enum {
    NONE,
    GOING_TO_ZERO,
    GOING_BY_POINTS,
    HEAD_MOVING
} workState;

enum {
    UP,
    DOWN
} headPosition;

const float SPEED_X = 400; // для одного движка, больше 540 уже клинит иногда
const float SPEED_Y = 400; // для одного движка, больше 540 уже клинит иногда
const float SPEED_Z = 300; // для двух движков в параллель, больше 450 уже клинит иногда

bool calibrationModeActive = false;
bool workModeActive = false;

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
int currentPoint = 0;

// активация режима выполнения каждого шага по нажатию кнопки
const bool NEXT_STEP_FROM_BUTTON = false;
bool awaitButton = true;

void detectCurrentMode() {
    if (digitalRead(sw2) == HIGH) {
        currentMode = CALIBRATION;
    } else {
        currentMode = WORK;
    }
}

void detectCurrentAxis() {
    if (digitalRead(sw1_1) == HIGH) {
        currentAxis = X;
    } else if (digitalRead(sw1_2) == HIGH) {
        currentAxis = Y;
    } else {
        currentAxis = Z;
    }
}

void forward() {
    if (currentAxis == X) {
        if (!stepperX.tick()) {
            stepperX.setSpeed(SPEED_X);
        }
    } else if (currentAxis == Y) {
        if (!stepperY.tick()) {
            stepperY.setSpeed(SPEED_Y);
        }
    } else if (currentAxis == Z) {
        if (!stepperZ.tick()) {
            stepperZ.setSpeed(SPEED_Z);
        }
    }
}

void back() {
    if (currentAxis == X) {
        if (!stepperX.tick()) {
            stepperX.setSpeed(-SPEED_X);
        }
    } else if (currentAxis == Y) {
        if (!stepperY.tick()) {
            stepperY.setSpeed(-SPEED_Y);
        }
    } else if (currentAxis == Z) {
        if (!stepperZ.tick()) {
            stepperZ.setSpeed(-SPEED_Z);
        }
    }
}

void stop() {
    if (currentMode == CALIBRATION) {
        if (currentAxis == X) {
            stepperX.brake();
        } else if (currentAxis == Y) {
            stepperY.brake();
        } else if (currentAxis == Z) {
            stepperZ.brake();
        }
    } else if (currentMode == WORK) {
        stepperX.brake();
        stepperY.brake();
        stepperZ.brake();
    }
}

void setZero() {
    if (currentAxis == X) {
        stepperX.reset();
    } else if (currentAxis == Y) {
        stepperY.reset();
    } else if (currentAxis == Z) {
        stepperZ.reset();
    }
}

bool goToZero() {
    bool xZeroReached = false;
    bool yZeroReached = false;
    // bool zZeroReached = false;
    bool zeroReached = false;

    // Если одновременно включить 4 движка, то питание проседает,
    // поэтому сначала выгоняю в 0 оси X, Y, а потом Z.
    // Да и поскольку сейчас 0 у Z это нижняя точка, то надо сначала увести стол.

    // если голова поднята, то выгоняем оси, иначе сначала поднимаем голову
    if (headPosition == UP) {
        if (stepperX.getCurrent() != 0) {
            if (!stepperX.tick()) {
                stepperX.setTarget(0);
            }
        } else {
            stepperX.brake();
            xZeroReached = true;
        }

        if (stepperY.getCurrent() != 0) {
            if (!stepperY.tick()) {
                stepperY.setTarget(0);
            }
        } else {
            stepperY.brake();
            yZeroReached = true;
        }
    } else {
        moveHead(0);
    }

    if (xZeroReached && yZeroReached) {
        bool headMoved = moveHead(1);
        if (headMoved) {
            zeroReached = true;
        }
    }

    return zeroReached;
}

bool goToPoint(int x, int y) {
    bool xReached = false;
    bool yReached = false;
    bool pointReached = false;

    if (stepperX.getCurrent() != x) {
        if (!stepperX.tick()) {
            stepperX.setTarget(x);
        }
    } else {
        stepperX.brake();
        xReached = true;
    }

    if (stepperY.getCurrent() != y) {
        if (!stepperY.tick()) {
            stepperY.setTarget(y);
        }
    } else {
        stepperY.brake();
        yReached = true;
    }

    if (xReached && yReached) {
        pointReached = true;
    }

    return pointReached;
}

bool moveHead(int z) {
    long upPosition = -500;
    long downPosition = 0;
    bool headMoved = false;

    if (z == 0) {
        if (stepperZ.getCurrent() != upPosition) {
            if (!stepperZ.tick()) {
                stepperZ.setTarget(upPosition);
            }
        } else {
            stepperZ.brake();
            headMoved = true;
            headPosition = UP;
        }
    } else if (z == 1) {
        if (stepperZ.getCurrent() != downPosition) {
            if (!stepperZ.tick()) {
                stepperZ.setTarget(downPosition);
            }
        } else {
            stepperZ.brake();
            headMoved = true;
            headPosition = DOWN;
        }
    }

    return headMoved;
}

void setupCalibrationMode() {
    if (calibrationModeActive) {
        return;
    }

    stepperX.setRunMode(KEEP_SPEED);
    stepperY.setRunMode(KEEP_SPEED);
    stepperZ.setRunMode(KEEP_SPEED);

    stepperX.setAcceleration(0);
    stepperY.setAcceleration(0);
    stepperZ.setAcceleration(0);

    stepperX.autoPower(true);
    stepperY.autoPower(true);
    stepperZ.autoPower(true);

    calibrationModeActive = true;
    workModeActive = false;
}

void setupWorkMode() {
    if (workModeActive) {
        return;
    }

    stepperX.setRunMode(FOLLOW_POS);
    stepperY.setRunMode(FOLLOW_POS);
    stepperZ.setRunMode(FOLLOW_POS);

    stepperX.setMaxSpeed(SPEED_X);
    stepperY.setMaxSpeed(SPEED_Y);
    stepperZ.setMaxSpeed(SPEED_Z);

    stepperX.setAcceleration(0);
    stepperY.setAcceleration(0);
    stepperZ.setAcceleration(0);

    stepperX.autoPower(true);
    stepperY.autoPower(true);
    stepperZ.autoPower(true);        

    workModeActive = true;
    calibrationModeActive = false;
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

    workState = NONE;
    headPosition = DOWN;
    Serial.println("ready");
}

void loop()	{
    detectCurrentMode();

    if (currentMode == CALIBRATION) {
        setupCalibrationMode();
        detectCurrentAxis();

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

    } else if (currentMode == WORK) {
        setupWorkMode();

        // кнопка 2
        if (digitalRead(bt2) == HIGH) {
            // в режиме простоя - поднять/опустить голову
            if (workState == NONE) {
                // защита от дребезга контактов
                delay(10);
                if (digitalRead(bt2) == HIGH) {
                    // ждем когда кнопка будет отжата
                    while (true) {
                        if (digitalRead(bt2) == LOW) {
                            delay(10);
                            if (digitalRead(bt2) == LOW) {
                                workState = HEAD_MOVING;
                                break;
                            }
                        }
                    }
                }
            // в остальных режимах - полная остановка
            } else {
                stop();
                currentPoint = 0;
                awaitButton = true;
                workState = NONE;
                delay(2000); // пауза для того, чтобы кнопка сразу не сработала повторно                
            }
        // кнопка 1 - пуск движения по массиву точек
        } else if (digitalRead(bt1) == HIGH) {
            if (awaitButton) {
                delay(10);
                if (digitalRead(bt1) == HIGH) {
                    while (true) {
                        if (digitalRead(bt1) == LOW) {
                            delay(10);
                            if (digitalRead(bt1) == LOW) {
                                workState = GOING_BY_POINTS;
                                awaitButton = false;
                                break;
                            }
                        }
                    }
                }
            }
        // кнопка 3
        } else if (digitalRead(bt3) == HIGH) {
            // в режиме простоя - отвод осей в 0
            if (workState == NONE) {
                workState = GOING_TO_ZERO;
            }
        }

        if (workState == GOING_TO_ZERO) {
            bool zeroReached = goToZero();
            
            if (zeroReached) {
                workState = NONE;
            }
        } else if (workState == GOING_BY_POINTS) {
            // если не ждем нажатия кнопки, то работаем
            if (!awaitButton) {
                int z = POINTS[currentPoint][0];
                bool headMoved = moveHead(z);

                if (headMoved) {
                    int x = POINTS[currentPoint][1];
                    int y = POINTS[currentPoint][2];
                    bool pointReached = goToPoint(x, y);

                    if (pointReached) {
                        Serial.print(x);
                        Serial.print(" ");
                        Serial.print(y);
                        Serial.println(" point reached");

                        currentPoint++;
                        if (currentPoint == POINTS_SIZE) {
                            currentPoint = 0;
                            awaitButton = true;
                            workState = GOING_TO_ZERO;
                            Serial.println("End of POINTS array");
                            Serial.println("Going to zero");
                        }

                        // если включен режим "шаг по нажатию кнопки", то сбрасываем флаг ожидания
                        if (NEXT_STEP_FROM_BUTTON) {
                            awaitButton = true;
                        }
                    }
                }
            }
        } else if (workState == HEAD_MOVING) {
            bool headMoved = false;

            if (headPosition == DOWN) {
                headMoved = moveHead(0);
            } else if (headPosition == UP) {
                headMoved = moveHead(1);
            }

            if (headMoved) {
                workState = NONE;
            }
        }
    }

}