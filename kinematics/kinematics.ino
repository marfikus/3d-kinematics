#include <GyverStepper.h>

GStepper<STEPPER4WIRE> stepperX(2048, 13, 11, 12, 10);
GStepper<STEPPER4WIRE> stepperY(2048, 9, 7, 8, 6);
GStepper<STEPPER4WIRE> stepperZ(2048, 5, 3, 4, 2);

// BT - кнопка, SW - тумблер
const byte BT_1 = A0; // кнопка без фиксации
const byte BT_2 = A1; // кнопка без фиксации
// трёхпозиционный тумблер для переключения осей в режиме калибровки:
const byte SW_1_1 = A2;
const byte SW_1_2 = A3;
// двухпозиционный тумблер для переключения режимов (калибровка/работа):
const byte SW_2 = A4; 
const byte BT_3 = A5; // кнопка без фиксации

// активация serial
const bool SERIAL_ENABLED = true;
const long SERIAL_SPEED = 9600;

// скорости движения осей
const float SPEED_X = 400; // для одного движка, больше 540 уже клинит иногда
const float SPEED_Y = 400;
const float SPEED_Z = 300; // для двух движков в параллель, больше 450 уже клинит иногда

// активация коррекции погрешности механики (внесение поправки в координаты)
const bool CORRECTION_ENABLED = false;
// величины корректировок
const int X_CORRECTION = 50;
const int Y_CORRECTION = 0;
// величина начального сдвига для определения последних направлений
const int SHIFT = 500;

// верхняя и нижняя позиции головы (ось z)
const int UP_HEAD_POSITION = -500;
const int DOWN_HEAD_POSITION = 0;

// активация режима движения к каждой точке массива по нажатию кнопки
const bool NEXT_POINT_FROM_BUTTON = false;

// массив точек:
// первый элемент - положение головки(0 - поднята, 1 - опущена)
// второй элемент - координата х
// третий элемент - координата у

// прямоугольник:
/*const int POINTS[][3] = {
    {0, 3000, 1000},
    {1, 3000, 2000},
    {1, 4000, 2000},
    {1, 4000, 1000},
    {1, 3000, 1000}
};*/

//прямоугольный треугольник:
/*int POINTS[][3] = {
    {0, 1700, 600},
    {1, 1700, 3600},
    {1, 4700, 600},
    {1, 1700, 600},
};*/

// восьмиугольник:
/*const int POINTS[][3] = {
    {0, 2000, 1000},
    {1, 3000, 1000},
    {1, 4000, 2000},
    {1, 4000, 3000},
    {1, 3000, 4000},
    {1, 2000, 4000},
    {1, 1000, 3000},
    {1, 1000, 2000},
    {1, 2000, 1000},
};*/

// вычисление количества элементов в массиве
// const int POINTS_SIZE = sizeof(POINTS) / sizeof(POINTS[0]);
int currentPoint = 0;

// круг (массив заполняется при запуске в generateCirclePointsArray)
// центр и радиус
const int X0 = 3000;
const int Y0 = 3000;
const int RADIUS = 1000;
// шаг координат
const int STEP = 100;

const int POINTS_SIZE = (RADIUS / STEP) * 4;
int POINTS[POINTS_SIZE][3];

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
    AWAIT_COMMAND,
    GOING_TO_ZERO,
    GOING_BY_POINTS,
    HEAD_MOVING,
    LAST_DIRECTIONS_DETECTING
} workState;

enum {
    UP,
    DOWN
} headPosition;

enum Directions {
    RIGHT,
    LEFT
};
Directions xLastDirection;
Directions yLastDirection;
Directions xNeedDirection;
Directions yNeedDirection;

enum {
    START,
    ZERO_REACHED,
    SHIFT_REACHED
} detectLastDirectionsState;

bool lastDirectionsDetected = false;
bool correctedValues = false;
int correctedX = 0;
int correctedY = 0;
int xCorrection = 0;
int yCorrection = 0;

bool calibrationModeActive = false;
bool workModeActive = false;
bool awaitButton = true;

struct SquareEquationResult {
    byte statusCode;
    long x1;
    long x2;
};

SquareEquationResult solveSquareEquation(long a, long b, long c) {
    byte statusCode = 0;
    long x1 = 0;
    long x2 = 0;
    long d = (b * b) - 4 * a * c;
    // Serial.println(d);

    if (d > 0) {
        // два корня
        statusCode = 2;
        long sqrtD = sqrt(d);
        // Serial.println(sqrtD);
        x1 = round((-b + sqrtD) / 2 * a);
        x2 = round((-b - sqrtD) / 2 * a);
    } else if (d == 0) {
        // один корень
        statusCode = 1;
        x1 = round(-b / 2 * a);
    } else {
        // нет корней
    }

    // Serial.println(x1);
    // Serial.println(x2);

    return (SquareEquationResult) {statusCode, x1, x2};
}

void generateCirclePointsArray(int x0, int y0, int r, int step) {
    int xMin = x0 - r;
    int xMax = x0 + r;
    int yMin = y0 - r;
    int yMax = y0 + r;

    // int pointsSize = (r / step) * 4;
    // Serial.println(pointsSize);
    // int POINTS[pointsSize][3];



    int currentPos = 0;
    for (int x = xMin, y = y0; x < x0; x += step, y -= step) {
        POINTS[currentPos][0] = 1;
        POINTS[currentPos][1] = x;
        POINTS[currentPos][2] = y;
        currentPos++;
    }
    // Serial.println(currentPos);
    for (int x = x0, y = yMin; x < xMax; x += step, y += step) {
        POINTS[currentPos][0] = 1;
        POINTS[currentPos][1] = x;
        POINTS[currentPos][2] = y;
        currentPos++;
    }
    // Serial.println(currentPos);
    for (int x = xMax, y = y0; x > x0; x -= step, y += step) {
        POINTS[currentPos][0] = 1;
        POINTS[currentPos][1] = x;
        POINTS[currentPos][2] = y;
        currentPos++;
    }
    // Serial.println(currentPos);
    for (int x = x0, y = yMax; x > xMin; x -= step, y -= step) {
        POINTS[currentPos][0] = 1;
        POINTS[currentPos][1] = x;
        POINTS[currentPos][2] = y;
        currentPos++;
    }
    // Serial.println(currentPos);

    POINTS[0][0] = 0;

/*    for (int i = 0; i < pointsSize; i++) {
        Serial.print(POINTS[i][0]);
        Serial.print(" ");
        Serial.print(POINTS[i][1]);
        Serial.print(" ");
        Serial.println(POINTS[i][2]);
    }*/
}

void detectCurrentMode() {
    if (digitalRead(SW_2) == HIGH) {
        currentMode = CALIBRATION;
    } else {
        currentMode = WORK;
    }
}

void detectCurrentAxis() {
    if (digitalRead(SW_1_1) == HIGH) {
        currentAxis = X;
    } else if (digitalRead(SW_1_2) == HIGH) {
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

bool goToZero(bool moveHeadDown) {
    bool xZeroReached = false;
    bool yZeroReached = false;
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
        // если нужно опускаем голову
        if (moveHeadDown) {
            bool headMoved = moveHead(1);
            if (headMoved) {
                zeroReached = true;
            }
        } else {
            zeroReached = true;
        }
    }

    return zeroReached;
}

// перегрузка функции, чтобы можно было вызвать без параметров
bool goToZero() {
    return goToZero(true);
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
    bool headMoved = false;

    if (z == 0) {
        if (stepperZ.getCurrent() != UP_HEAD_POSITION) {
            if (!stepperZ.tick()) {
                stepperZ.setTarget(UP_HEAD_POSITION);
            }
        } else {
            stepperZ.brake();
            headMoved = true;
            headPosition = UP;
        }
    } else if (z == 1) {
        if (stepperZ.getCurrent() != DOWN_HEAD_POSITION) {
            if (!stepperZ.tick()) {
                stepperZ.setTarget(DOWN_HEAD_POSITION);
            }
        } else {
            stepperZ.brake();
            headMoved = true;
            headPosition = DOWN;
        }
    }

    return headMoved;
}

void detectLastDirections() {
    // Алгоритм такой: 
    //  поднимаем голову, выгоняем оси в 0 
    //  и делаем небольшой сдвиг туда-обратно

    if (detectLastDirectionsState == START) {
        bool zeroReached = goToZero(false); // гоним в 0 без опускания головы
        if (zeroReached) {
            detectLastDirectionsState = ZERO_REACHED;
        }
    } else if (detectLastDirectionsState == ZERO_REACHED) {
        bool shiftReached = goToPoint(SHIFT, SHIFT);
        if (shiftReached) {
            detectLastDirectionsState = SHIFT_REACHED;
        }
    } else if (detectLastDirectionsState == SHIFT_REACHED) {
        bool zeroReached = goToZero(false);
        if (zeroReached) {
            detectLastDirectionsState = START;
            xLastDirection = LEFT;
            yLastDirection = LEFT;
            lastDirectionsDetected = true;
        }        
    }
}

void updateLastDirections() {
    xLastDirection = xNeedDirection;
    yLastDirection = yNeedDirection;
}

int correctValue(char axis, int newValue) {
    int correctedNewValue = newValue;

    if (axis == 'x') {
        int currentValue = stepperX.getCurrent() + xCorrection;
        if (newValue > currentValue) {
            xNeedDirection = RIGHT;
        } else if (newValue < currentValue) {
            xNeedDirection = LEFT;
        } else {
            xNeedDirection = xLastDirection;
        }

        if (xNeedDirection != xLastDirection) {
            if (xNeedDirection == RIGHT) {
                correctedNewValue = newValue + X_CORRECTION;
                xCorrection = -X_CORRECTION;
            } else if (xNeedDirection == LEFT) {
                correctedNewValue = newValue - X_CORRECTION;
                xCorrection = X_CORRECTION;
            }
        }

    } else if (axis == 'y') {
        int currentValue = stepperY.getCurrent() + yCorrection;
        if (newValue > currentValue) {
            yNeedDirection = RIGHT;
        } else if (newValue < currentValue) {
            yNeedDirection = LEFT;
        } else {
            yNeedDirection = yLastDirection;
        }

        if (yNeedDirection != yLastDirection) {
            if (yNeedDirection == RIGHT) {
                correctedNewValue = newValue + Y_CORRECTION;
                yCorrection = -Y_CORRECTION;
            } else if (yNeedDirection == LEFT) {
                correctedNewValue = newValue - Y_CORRECTION;
                yCorrection = Y_CORRECTION;
            }
        }
    }

    return correctedNewValue;
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
	pinMode(BT_1, INPUT);
	pinMode(BT_2, INPUT);
    pinMode(SW_1_1, INPUT);
    pinMode(SW_1_2, INPUT);
    pinMode(SW_2, INPUT);
    pinMode(BT_3, INPUT);

    workState = AWAIT_COMMAND;
    headPosition = DOWN;

    if (SERIAL_ENABLED) {
        Serial.begin(SERIAL_SPEED);
        Serial.println("ready");
    }

    // SquareEquationResult result = solveSquareEquation(1, 6, 9);
    SquareEquationResult result = solveSquareEquation(1, 6000, 9000000);
    // SquareEquationResult result = solveSquareEquation(1, 6000, 8810000);
    // SquareEquationResult result = solveSquareEquation(1, 6000, 8640000);
    Serial.println(result.statusCode);
    Serial.println(result.x1);
    Serial.println(result.x2);

/*    generateCirclePointsArray(X0, Y0, RADIUS, STEP);
    for (int i = 0; i < POINTS_SIZE; i++) {
        Serial.print(POINTS[i][0]);
        Serial.print(" ");
        Serial.print(POINTS[i][1]);
        Serial.print(" ");
        Serial.println(POINTS[i][2]);
    }*/
}

void loop()	{
    detectCurrentMode();

    if (currentMode == CALIBRATION) {
        setupCalibrationMode();
        detectCurrentAxis();

    	if (digitalRead(BT_1) == HIGH) {
            forward();
    	} else if (digitalRead(BT_2) == HIGH) {
            back();
    	} else {
            stop();
    	}

        if (digitalRead(BT_3) == HIGH) {
            setZero();
        }

    } else if (currentMode == WORK) {
        setupWorkMode();

        // кнопка 2
        if (digitalRead(BT_2) == HIGH) {
            // в режиме простоя - поднять/опустить голову
            if (workState == AWAIT_COMMAND) {
                // защита от дребезга контактов
                delay(10);
                if (digitalRead(BT_2) == HIGH) {
                    // ждем когда кнопка будет отжата
                    while (true) {
                        if (digitalRead(BT_2) == LOW) {
                            delay(10);
                            if (digitalRead(BT_2) == LOW) {
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
                correctedValues = false;
                xCorrection = 0;
                yCorrection = 0;
                awaitButton = true;
                lastDirectionsDetected = false;
                workState = AWAIT_COMMAND;
                delay(2000); // пауза для того, чтобы кнопка сразу не сработала повторно                
            }
        // кнопка 1 - пуск движения по массиву точек
        } else if (digitalRead(BT_1) == HIGH) {
            if (awaitButton) {
                delay(10);
                if (digitalRead(BT_1) == HIGH) {
                    while (true) {
                        if (digitalRead(BT_1) == LOW) {
                            delay(10);
                            if (digitalRead(BT_1) == LOW) {
                                if (CORRECTION_ENABLED) {
                                    if (!lastDirectionsDetected) {
                                        workState = LAST_DIRECTIONS_DETECTING;
                                        detectLastDirectionsState = START;
                                    } else {
                                        workState = GOING_BY_POINTS;
                                    }
                                } else {
                                    workState = GOING_BY_POINTS;
                                }
                                awaitButton = false;
                                break;
                            }
                        }
                    }
                }
            }
        // кнопка 3
        } else if (digitalRead(BT_3) == HIGH) {
            // в режиме простоя - отвод осей в 0
            if (workState == AWAIT_COMMAND) {
                workState = GOING_TO_ZERO;
            }
        }

        if (workState == GOING_TO_ZERO) {
            bool zeroReached = goToZero();
            
            if (zeroReached) {
                workState = AWAIT_COMMAND;
            }

        } else if (workState == LAST_DIRECTIONS_DETECTING) {
            detectLastDirections();
            if (lastDirectionsDetected) {
                workState = GOING_BY_POINTS;
                correctedValues = false;
                xCorrection = 0;
                yCorrection = 0;
            }

        } else if (workState == GOING_BY_POINTS) {
            // если не ждем нажатия кнопки, то работаем
            if (!awaitButton) {
                int z = POINTS[currentPoint][0];
                bool headMoved = moveHead(z);

                if (headMoved) {
                    int x = POINTS[currentPoint][1];
                    int y = POINTS[currentPoint][2];

                    if (CORRECTION_ENABLED) {
                        if (!correctedValues) {
                            correctedX = correctValue('x', x);
                            correctedY = correctValue('y', y);
                            correctedValues = true;
                        }
                        x = correctedX;
                        y = correctedY;
                    }

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
                            lastDirectionsDetected = false;
                            Serial.println("End of POINTS array");
                            Serial.println("Going to zero");
                        }

                        // если включен режим движения по нажатию кнопки, то сбрасываем флаг ожидания
                        if (NEXT_POINT_FROM_BUTTON) {
                            awaitButton = true;
                        }
                        if (CORRECTION_ENABLED) {
                            updateLastDirections();
                            correctedValues = false;
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
                workState = AWAIT_COMMAND;
            }
        }
    }

}