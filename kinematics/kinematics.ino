#include <GyverStepper.h>

GStepper<STEPPER4WIRE> stepperX(2048, 5, 3, 4, 2);
/* подключение к драйверу:
in 1 - 2 
in 2 - 3
in 3 - 4
in 4 - 5
*/

GStepper<STEPPER4WIRE> stepperY(2048, 9, 7, 8, 6);
/* подключение к драйверу:
in 1 - 6 
in 2 - 7
in 3 - 8
in 4 - 9
*/

GStepper<STEPPER4WIRE> stepperZ(2048, 13, 11, 12, 10);
/* подключение к драйверу (тут на два драйвера параллельно):
in 1 - 10 
in 2 - 11
in 3 - 12
in 4 - 13
*/

// BT - кнопка, SW - тумблер
const byte BT_1 = A0; // кнопка без фиксации
const byte BT_2 = A1; // кнопка без фиксации
// трёхпозиционный тумблер для переключения осей в режиме калибровки:
const byte SW_1_1 = A2;
const byte SW_1_2 = A3;
// двухпозиционный тумблер для переключения режимов (калибровка/работа):
const byte SW_2 = A4; 
const byte BT_3 = A5; // кнопка без фиксации

// активация serial:
// Заметил, что включенный serial мешает рисовать окружности с большим 
// количеством точек(сбивается с координат).
const bool SERIAL_ENABLED = false;
const long SERIAL_SPEED = 9600;

// максимальные скорости движения осей:
// (реальные скорости будут вычисляться в работе для равномерного движения)
// для одного движка, больше 540 клинит иногда, для двух в параллель больше 450
const float MAX_SPEED_X = 400;
const float MAX_SPEED_Y = 400;
const float MAX_SPEED_Z = 300;

// активация коррекции погрешности механики (внесение поправки в координаты)
const bool CORRECTION_ENABLED = true;
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
int POINTS[][3] = {
    {0, 3000, 1000},
    {1, 3000, 2000},
    {1, 4000, 2000},
    {1, 4000, 1000},
    {1, 3000, 1000}
};

//прямоугольный треугольник:
/*int POINTS[][3] = {
    {0, 1700, 600},
    {1, 1700, 3600},
    {1, 4700, 600},
    {1, 1700, 600},
};*/

// равнобедренный треугольник:
/*int POINTS[][3] = {
    {0, 1000, 1500},
    {1, 2500, 500},
    {1, 2500, 2500},
    {1, 1000, 1500},
};*/

// прямоугольный треугольник 2:
/*int POINTS[][3] = {
    {0, 500, 1000},
    {1, 2000, 1000},
    {1, 2000, 0},
    {1, 500, 1000},
};*/

// восьмиугольник:
/*int POINTS[][3] = {
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

// вычисление количества элементов в массиве:
// (только если массив заполняется вручную (варианты выше),
// иначе (для генерируемого массива) размер задается ниже!)
const int POINTS_SIZE = sizeof(POINTS) / sizeof(POINTS[0]);


// окружность или если использовать generateCirclePointsArray2,
// то можно получить многоугольник с заданным числом вершин
// (массив заполняется при запуске в generateCirclePointsArray
// или в generateCirclePointsArray2 (раскомментировать нужное в setup)):
// центр и радиус:
const int X0 = 3000;
const int Y0 = 1000;
const int RADIUS = 1000;
// шаг координат (для generateCirclePointsArray):
const int STEP = 50;
// количество вершин (для generateCirclePointsArray2):
// (при VERTEX_NUMBER >= 90 и включенном Serial сбивается с координат)
// const int VERTEX_NUMBER = 3; // получим равносторонний треугольник
const int VERTEX_NUMBER = 60; // получим вполне ровную окружность
// размер массива: 
// для generateCirclePointsArray:
// const int POINTS_SIZE = ((RADIUS / STEP) * 4) + 1;
// для generateCirclePointsArray2:
// const int POINTS_SIZE = VERTEX_NUMBER + 1;
// создание массива для окружности:
// int POINTS[POINTS_SIZE][3];


// звезда или другая подобная фигура с двумя радиусами (меньший и больший)
// (массив заполняется при запуске в generateStarPointsArray
// (раскомментировать нужное в setup)):
// количество лучей:
const int RAYS_NUMBER = 5;
// центр:
// const int X0 = 1000;
// const int Y0 = 1000;
// больший радиус:
const int R1 = 1000;
// меньший радиус:
const int R2 = 400;
// размер массива и его создание:
// const int POINTS_SIZE = RAYS_NUMBER * 2 + 1;
// int POINTS[POINTS_SIZE][3];


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
int currentPoint = 0;
bool steppersSpeedCorrected = false;

struct SquareEquationCoefficients {
    long a;
    long b;
    long c;
};

struct SquareEquationResult {
    byte statusCode;
    long r1;
    long r2;
};

SquareEquationCoefficients getSquareEquationCoefficients(long x0, long y0, long x, long radius) {
    /*
    Получает коэффициенты квадратного уравнения 
    из имеющихся данных о окружности
    */

    long a = 1;
    long b = 2 * (-y0);
    long c = sq(-y0) - (sq(radius) - sq(x - x0));

    return (SquareEquationCoefficients) {a, b, c};   
}

SquareEquationResult solveSquareEquation(long a, long b, long c) {
    /*
    Решает квадратное уравнение
    */

    byte statusCode = 0;
    long r1 = 0;
    long r2 = 0;
    long d = (b * b) - 4 * a * c;

    if (d > 0) {
        // два корня
        statusCode = 2;
        long sqrtD = sqrt(d);
        r1 = round((-b + sqrtD) / 2 * a);
        r2 = round((-b - sqrtD) / 2 * a);
    } else if (d == 0) {
        // один корень
        statusCode = 1;
        r1 = round(-b / 2 * a);
    } else {
        // нет корней
    }

    return (SquareEquationResult) {statusCode, r1, r2};
}

void generateCirclePointsArray() {
    /*
    Заполняет массив точек, посредством решения 
    квадратных уравнений для точек одной четверти, 
    а потом симметрично заполняет остальные 
    */

    int xMin = X0 - RADIUS;
    int xMax = X0 + RADIUS;
    int yMin = Y0 - RADIUS;
    int yMax = Y0 + RADIUS;

    int pos1 = 0;
    int pos4 = (POINTS_SIZE - 1) - pos1;
    int pos3 = (POINTS_SIZE - 1) / 2;
    int pos2 = (POINTS_SIZE - 1) - pos3;

    for (int x = xMin, x2 = xMax; x < X0; x += STEP, x2 -= STEP) {
        POINTS[pos1][0] = 1;
        POINTS[pos1][1] = x;

        POINTS[pos3][0] = 1;
        POINTS[pos3][1] = x2;

        SquareEquationCoefficients coefs = getSquareEquationCoefficients(X0, Y0, x, RADIUS);
        SquareEquationResult result = solveSquareEquation(coefs.a, coefs.b, coefs.c);

        // заполнение всех четвертей за один проход
        // если 2 корня, то меньший ставим в текущую точку, а больший симметрично в верхнюю четверть,
        // ну и другие тоже симметрично (на графике сразу понятно)
        if (result.statusCode == 2) {
            POINTS[pos4][0] = 1;
            POINTS[pos4][1] = x;

            POINTS[pos2][0] = 1;
            POINTS[pos2][1] = x2;

            if (result.r1 < result.r2) {
                POINTS[pos1][2] = result.r1;
                POINTS[pos4][2] = result.r2;

                POINTS[pos2][2] = result.r1;
                POINTS[pos3][2] = result.r2;
            } else {
                POINTS[pos1][2] = result.r2;
                POINTS[pos4][2] = result.r1;

                POINTS[pos2][2] = result.r2;
                POINTS[pos3][2] = result.r1;
            }
        // если 1 корень, то ставим его в текущую точку
        } else if (result.statusCode == 1) {
            POINTS[pos1][2] = result.r1;

            POINTS[pos3][2] = result.r1;
        // если нет корней (что маловероятно), то
        } else if (result.statusCode == 0) {
            // если не первая точка, то ставим в неё значение из предыдущей, иначе Y0
            if (pos1 != 0) {
                POINTS[pos1][2] = POINTS[pos1 - 1][2];
                POINTS[pos2][2] = POINTS[pos2 - 1][2];
                POINTS[pos3][2] = POINTS[pos3 - 1][2];
                POINTS[pos4][2] = POINTS[pos4 - 1][2];
            } else {
                POINTS[pos1][2] = Y0;
                POINTS[pos2][2] = Y0;
                POINTS[pos3][2] = Y0;
                POINTS[pos4][2] = Y0;
            }
        }
        
        pos1++;
        pos4 = (POINTS_SIZE - 1) - pos1;
        pos3++;
        pos2 = (POINTS_SIZE - 1) - pos3;
    }

    // заполнение оставшихся крайних точек
    // (можно наверное и их в цикле заполнить, но пока не сообразил)
    POINTS[pos1][0] = 1;
    POINTS[pos1][1] = X0;
    POINTS[pos1][2] = yMin;

    POINTS[pos3][0] = 1;
    POINTS[pos3][1] = X0;
    POINTS[pos3][2] = yMax;

    // указываем, чтобы к первой точке двигался с поднятой головой
    POINTS[0][0] = 0;

    // заполняем последнюю точку, чтобы замкнуть круг
    POINTS[POINTS_SIZE - 1][0] = 1;
    POINTS[POINTS_SIZE - 1][1] = POINTS[0][1];
    POINTS[POINTS_SIZE - 1][2] = POINTS[0][2];
}

void generateCirclePointsArray2() {
    /*
    В общем делает то же самое, что и предыдущая функция, 
    но гораздо короче и наверное оптимальнее.
    Кроме того, играясь с константами в начале файла, можно получить
    разные многоугольники.
    */

    int angle = 0;
    int angleStep = round(360 / (POINTS_SIZE - 1));

    for (int i = 0; i < POINTS_SIZE; i++) {

        POINTS[i][0] = 1;
        POINTS[i][1] = RADIUS * cos(radians(angle)) + X0;
        POINTS[i][2] = RADIUS * sin(radians(angle)) + Y0;

        angle += angleStep;
    }

    POINTS[0][0] = 0;
}

void generateStarPointsArray() {
    /*
    Генерирует точки звезды с заданным количеством лучей.
    Больший радиус определяет крайнюю точку луча, 
    меньший - основание. 
    Играясь с константами в начале файла, можно получить разные фигуры.
    */

    int angle1 = 0;
    int angle2 = round(360 / (POINTS_SIZE - 1));
    int angleStep = angle2 * 2;

    for (int i = 0, j = 1; j < POINTS_SIZE; i += 2, j += 2) {

        POINTS[i][0] = 1;
        POINTS[i][1] = R1 * cos(radians(angle1)) + X0;
        POINTS[i][2] = R1 * sin(radians(angle1)) + Y0;

        POINTS[j][0] = 1;
        POINTS[j][1] = R2 * cos(radians(angle2)) + X0;
        POINTS[j][2] = R2 * sin(radians(angle2)) + Y0;

        angle1 += angleStep;
        angle2 += angleStep;
    }

    POINTS[POINTS_SIZE - 1][0] = 1;
    POINTS[POINTS_SIZE - 1][1] = POINTS[0][1];
    POINTS[POINTS_SIZE - 1][2] = POINTS[0][2];

    POINTS[0][0] = 0;
}

void detectCurrentMode() {
    /*
    Определяет текущий режим работы по положению тумблера
    */

    if (digitalRead(SW_2) == HIGH) {
        currentMode = CALIBRATION;
    } else {
        currentMode = WORK;
    }
}

void detectCurrentAxis() {
    /*
    Определяет текущую ось по положению тумблера (в режиме калибровки)
    */

    if (digitalRead(SW_1_1) == HIGH) {
        currentAxis = X;
    } else if (digitalRead(SW_1_2) == HIGH) {
        currentAxis = Y;
    } else {
        currentAxis = Z;
    }
}

void forward() {
    /*
    Движение вперед для текущей оси (в режиме калибровки)
    */

    if (currentAxis == X) {
        if (!stepperX.tick()) {
            stepperX.setSpeed(MAX_SPEED_X);
        }
    } else if (currentAxis == Y) {
        if (!stepperY.tick()) {
            stepperY.setSpeed(MAX_SPEED_Y);
        }
    } else if (currentAxis == Z) {
        if (!stepperZ.tick()) {
            stepperZ.setSpeed(MAX_SPEED_Z);
        }
    }
}

void back() {
    /*
    Движение назад для текущей оси (в режиме калибровки)
    */

    if (currentAxis == X) {
        if (!stepperX.tick()) {
            stepperX.setSpeed(-MAX_SPEED_X);
        }
    } else if (currentAxis == Y) {
        if (!stepperY.tick()) {
            stepperY.setSpeed(-MAX_SPEED_Y);
        }
    } else if (currentAxis == Z) {
        if (!stepperZ.tick()) {
            stepperZ.setSpeed(-MAX_SPEED_Z);
        }
    }
}

void stop() {
    /*
    Остановка текущей оси (в режиме калибровки)
    или остановка всех осей (в режиме работы)
    */

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
    /*
    Установка нуля для текущей оси (в режиме калибровки)
    */

    if (currentAxis == X) {
        stepperX.reset();
    } else if (currentAxis == Y) {
        stepperY.reset();
    } else if (currentAxis == Z) {
        stepperZ.reset();
    }
}

bool goToZero(bool moveHeadDown) {
    /*
    Движение осей к текущим нулям (в режиме работы)

    Флаг moveHeadDown позволяет при необходимости 
    не опускать голову в конце процедуры.

    Если одновременно включить 4 движка, то питание проседает,
    поэтому сначала выгоняю в 0 оси X, Y, а потом Z.
    Да и поскольку сейчас 0 у Z это нижняя точка, то надо сначала увести стол.
    */

    bool xyZeroReached = false;
    bool zeroReached = false;

    // если голова поднята, то выгоняем оси, иначе сначала поднимаем голову
    if (headPosition == UP) {
        xyZeroReached = goToPoint(0, 0, true);
    } else {
        moveHead(0);
    }

    if (xyZeroReached) {
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

bool goToPoint(int x, int y, bool resetSpeed) {
    /*
    Движение к указанной координате (в режиме работы)

    Флаг resetSpeed передаётся в correctSteppersSpeed,
    чтобы при необходимости можно было сбросить скорости движков.
    */

    bool xReached = false;
    bool yReached = false;
    bool pointReached = false;

    if (!steppersSpeedCorrected) {
        correctSteppersSpeed(x, y, resetSpeed);
    }

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
        steppersSpeedCorrected = false;
    }

    return pointReached;
}

bool goToPoint(int x, int y) {
    return goToPoint(x, y, false);
}

bool moveHead(int z) {
    /*
    Подъём/опускание головы (в режиме работы)
    0 - поднять
    1 - опустить
    */

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
    /*    
    Определяет последние направления движения осей.
    Необходимо для дальнейшей корректировки координат
    (если она активирована).

    Алгоритм: 
        Поднимаем голову, 
        выгоняем оси в ноль 
        и делаем небольшой сдвиг туда-обратно
    */

    if (detectLastDirectionsState == START) {
        bool zeroReached = goToZero(false); // гоним в 0 без опускания головы в конце
        if (zeroReached) {
            detectLastDirectionsState = ZERO_REACHED;
        }
    } else if (detectLastDirectionsState == ZERO_REACHED) {
        bool shiftReached = goToPoint(SHIFT, SHIFT, true);
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
    /*
    Обновить последние направления движения осей.
    Необходимо для корректировки координат
    (если она активирована).
    */

    xLastDirection = xNeedDirection;
    yLastDirection = yNeedDirection;
}

int correctValue(char axis, int newValue) {
    /*
    Коррекция значений координат.
    Алгоритм:
        Определяем требуемое направление движения оси
        Если требуемое направление противоположно последнему направлению,
        то нужно подкорректировать значение 
        (добавить немного шагов на "перекладывание" механики из 
        стороны в сторону (есть небольшой люфт)).
        Иначе, если требуемое направление сонаправленно с последним направлением,
        то коррекция не требуется.
    */

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

void correctSteppersSpeed(int x, int y, bool resetSpeed) {
    /*
    Коррекция скоростей движения осей X, Y
    Необходима для равномерного движения осей к указанной точке.
    Например, когда выполняется рисование, одна из осей дойдёт до 
    своей точки раньше и остановится. В этом случае будет 
    "срезан" угол, что недопустимо.
    Алгоритм:
        Определяем расстояния, которые необходимо пройти осям.
        Выбираем из них наибольшее и вычисляем время необходимое
        для преодоления этого расстояния.
        Затем вычисляем под это время скорость движения другой оси,
        то есть замедляем её настолько, чтобы она двигалась 
        синхронно с первой.
        Ну и потом задаём эти значения движкам.

    Флаг resetSpeed позволяет сбросить значения скоростей в максимальные,
    что указываются в настройках в начале файла.
    */

    float trueSpeedX = MAX_SPEED_X;
    float trueSpeedY = MAX_SPEED_Y;

    if (!resetSpeed) {
        int diffX = abs(stepperX.getCurrent() - x);
        int diffY = abs(stepperY.getCurrent() - y);

        if ((diffX == 0) || (diffY == 0) || (diffX == diffY)) {
            trueSpeedX = MAX_SPEED_X;
            trueSpeedY = MAX_SPEED_Y;

        } else if (diffX > diffY) {
            float timeX = (float)diffX / MAX_SPEED_X;
            trueSpeedY = (float)diffY / timeX;

        } else if (diffY > diffX) {
            float timeY = (float)diffY / MAX_SPEED_Y;
            trueSpeedX = (float)diffX / timeY;
        }
    }

    stepperX.setMaxSpeed(trueSpeedX);
    stepperY.setMaxSpeed(trueSpeedY);
    steppersSpeedCorrected = true;
}

void setupCalibrationMode() {
    /*
    Установка конфигурации для режима калибровки.
    Благодаря флагу calibrationModeActive
    это выполняется один раз.
    */

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
    /*
    Установка конфигурации для режима работы
    Благодаря флагу workModeActive
    это выполняется один раз.
    */

    if (workModeActive) {
        return;
    }

    stepperX.setRunMode(FOLLOW_POS);
    stepperY.setRunMode(FOLLOW_POS);
    stepperZ.setRunMode(FOLLOW_POS);

    stepperX.setMaxSpeed(MAX_SPEED_X);
    stepperY.setMaxSpeed(MAX_SPEED_Y);
    stepperZ.setMaxSpeed(MAX_SPEED_Z);

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
    if (SERIAL_ENABLED) {
        Serial.begin(SERIAL_SPEED);
    }

	pinMode(BT_1, INPUT);
	pinMode(BT_2, INPUT);
    pinMode(SW_1_1, INPUT);
    pinMode(SW_1_2, INPUT);
    pinMode(SW_2, INPUT);
    pinMode(BT_3, INPUT);

    workState = AWAIT_COMMAND;
    headPosition = DOWN;

    // если раскомментировать одну из этих строк, то нужно также поправить настройки вверху!
    // generateCirclePointsArray();
    // generateCirclePointsArray2();
    // generateStarPointsArray();

/*    for (int i = 0; i < POINTS_SIZE; i++) {
        Serial.print(i);
        Serial.print(") ");
        Serial.print(POINTS[i][0]);
        Serial.print(" ");
        Serial.print(POINTS[i][1]);
        Serial.print(" ");
        Serial.println(POINTS[i][2]);
    }
*/
    Serial.println("ready");
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
                steppersSpeedCorrected = false;
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