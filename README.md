# Проект: Прохождение частиц через S-образный коридор

## О проекте

Проект моделирует движение частиц внутри S-образного коридора.
Частицы:

- взаимодействуют друг с другом через потенциал Леннарда-Джонса (см. [potentials.md](file:///home/lin/urfu_proj_cpp/docs/potentials.md))
- движутся под действием внешнего потенциального поля
- стартуют из `SpawnZone`
- должны попасть в `TargetZone`

Практическая цель проекта:

- запускать симуляцию интерактивно в окне SFML (GUI)
- запускать физику в `headless`-режиме
- подбирать параметры внешнего поля автоматически из Python-оптимизатора

Архитектура разделена на физическое ядро и GUI-оболочку, поэтому один и тот же набор параметров можно:

- прогонять автоматически без окна (в фоновом режиме)
- визуализировать и детально анализировать поведение частиц в реальном времени

## Текущее состояние

Реализованы следующие компоненты:

- Физическое ядро [SimulationCore](file:///home/lin/urfu_proj_cpp/include/SimulationCore.h)
- GUI-визуализатор [Simulation](file:///home/lin/urfu_proj_cpp/include/Simulation.h) на базе SFML 3
- Headless-режим (запуск с флагом `--headless`)
- Конфигурация параметров прогона через структуру [RunConfig](file:///home/lin/urfu_proj_cpp/include/RunConfig.h)
- Поддержка частичной/полной загрузки параметров и сцены из JSON
- Сохранение результатов headless-прогона в JSON-формате
- Метрика `quality` для оценки и сравнения эффективности прогонов
- Выделенный конфигурационный файл оптимизации [optimizer_config.json](file:///home/lin/urfu_proj_cpp/examples/optimizer_config.json)
- Скрипт численного градиентного спуска [gradient_descent.py](file:///home/lin/urfu_proj_cpp/scripts/gradient_descent.py)
- Скрипт визуализации результатов оптимизации [plot_optimization.py](file:///home/lin/urfu_proj_cpp/scripts/plot_optimization.py)

Основной сценарий работы:

1. Собрать исполняемый файл `simulation`.
2. Запустить оптимизатор на Python, который многократно выполняет `./build/simulation --headless` для оценки градиента.
3. Оптимизатор сохраняет лучший найденный [RunConfig](file:///home/lin/urfu_proj_cpp/include/RunConfig.h).
4. Проверить результаты визуально в GUI с помощью лучшей конфигурации.

## Технологии

- C++17
- SFML 3
- CMake
- Python 3
- `nlohmann/json`

## Структура проекта

```text
.
├── CMakeLists.txt
├── Makefile
├── main.cpp
├── README.md
├── README_old.md
├── docs/
│   └── potentials.md
├── examples/
│   ├── optimizer_config.json
│   └── run_config.json
├── include/
│   ├── CameraController.h
│   ├── Config.h
│   ├── Corridor.h
│   ├── HeadlessSimulation.h
│   ├── JsonRunIO.h
│   ├── Particle.h
│   ├── PhysicsForces.h
│   ├── Rect.h
│   ├── RunConfig.h
│   ├── Simulation.h
│   ├── SimulationCore.h
│   ├── SimulationStats.h
│   ├── Vector2.h
│   ├── VisualConfig.h
│   ├── Zone.h
│   └── nlohmann/json.hpp
├── scripts/
│   ├── gradient_descent.py
│   └── plot_optimization.py
├── runs/
│   └── gradient_descent/
│       ├── best_result.json
│       ├── best_run_config.json
│       ├── history.csv
│       └── summary.json
└── src/
    ├── CameraController.cpp
    ├── Corridor.cpp
    ├── HeadlessSimulation.cpp
    ├── JsonRunIO.cpp
    ├── Particle.cpp
    ├── PhysicsForces.cpp
    ├── Simulation.cpp
    ├── SimulationCore.cpp
    ├── SimulationStats.cpp
    └── Zone.cpp
```

## Архитектура

### Ядро (Core)

- [SimulationCore](file:///home/lin/urfu_proj_cpp/include/SimulationCore.h) содержит частицы, геометрию коридора, внутренние препятствия, зоны, статистику и физический шаг.
- [Particle](file:///home/lin/urfu_proj_cpp/include/Particle.h), [Corridor](file:///home/lin/urfu_proj_cpp/include/Corridor.h), [Zone](file:///home/lin/urfu_proj_cpp/include/Zone.h), [PhysicsForces](file:///home/lin/urfu_proj_cpp/include/PhysicsForces.h), [SimulationStats](file:///home/lin/urfu_proj_cpp/include/SimulationStats.h) полностью отделены от графической библиотеки SFML.

### GUI-визуализация

- [Simulation](file:///home/lin/urfu_proj_cpp/include/Simulation.h) отвечает за создание окна, обработку событий, камеру и отрисовку.
- [CameraController](file:///home/lin/urfu_proj_cpp/include/CameraController.h) и [VisualConfig](file:///home/lin/urfu_proj_cpp/include/VisualConfig.h) относятся исключительно к GUI-слою.

### Оптимизация и Ввод/Вывод (I/O)

- [RunConfig](file:///home/lin/urfu_proj_cpp/include/RunConfig.h) описывает runtime-параметры одного прогона.
- [JsonRunIO](file:///home/lin/urfu_proj_cpp/src/JsonRunIO.cpp) загружает конфигурационные JSON-файлы и сериализует результаты headless-прогонов.
- `HeadlessSimulationRunner` выполняет симуляцию в фоновом режиме до завершения или тайм-аута и возвращает метрики.

## Быстрый старт

```bash
# 1. Создать виртуальное окружение Python и установить зависимости
uv venv .venv
uv pip install --python .venv matplotlib pandas numpy

# 2. Собрать бинарник
make build

# 3. Запустить оптимизацию (результаты → runs/gradient_descent/)
make optimize

# 4. Посмотреть графики
make plot

# 5. Запустить симуляцию с лучшими параметрами в GUI
make gui-best
```

Список всех целей Makefile:

```bash
make help
```

## Сборка

### Требования

- C++17-совместимый компилятор (GCC, Clang, MSVC)
- CMake 3.16+
- Библиотека **SFML 3**
- Python 3 + пакетный менеджер [uv](https://github.com/astral-sh/uv)

### Linux / WSL

```bash
cmake -S . -B build
cmake --build build
```

If CMake не находит SFML автоматически:

```bash
cmake -S . -B build -DSFML_DIR=/path/to/SFML/lib/cmake/SFML
cmake --build build
```

### Windows / Visual Studio

1. Установите SFML 3 под соответствующий MSVC.
2. Откройте корень проекта в Visual Studio как CMake-проект.
3. При необходимости укажите путь:
   ```json
   "cmakeCommandArgs": "-DSFML_DIR=C:/Libs/SFML/lib/cmake/SFML"
   ```

## Запуск и Управление

### GUI-режим

```bash
./build/simulation
```

По умолчанию симуляция считывает параметры и геометрию сцены из [run_config.json](file:///home/lin/urfu_proj_cpp/examples/run_config.json).

Управление в окне GUI:

- `Left` — замедлить симуляцию (уменьшить количество шагов физики на кадр)
- `Right` — ускорить симуляцию (увеличить количество шагов физики на кадр)
- `P` — поставить симуляцию на паузу / снять с паузы
- `Space` — перезапустить симуляцию (сбросить состояние)
- `+` / `=` — приблизить камеру (zoom in)
- `-` / `Subtract` / `Hyphen` — отдалить камеру (zoom out)
- `Num0` / `0` — сбросить положение и масштаб камеры к стандартным
- Перетаскивание с зажатой **левой кнопкой мыши (LMB)** — перемещение камеры по сцене (pan)
- Прокрутка **колесика мыши** — приближение и отдаление камеры относительно текущей позиции курсора

Текущая скорость симуляции отображается в заголовке окна как число шагов физики `dt` на один кадр отрисовки.

### Запуск с JSON-конфигом

```bash
./build/simulation --input-json examples/run_config.json
```

## Runtime-параметры CLI

Бинарник поддерживает следующие аргументы командной строки:

- `--input-json <path>` — загрузить конфигурацию симуляции из JSON
- `--output-json <path>` — записать результаты headless-прогона в JSON-файл
- `--headless` — запустить в фоновом режиме без открытия графического окна
- `--max-time <float>` — максимальное время моделирования в секундах (только для `--headless`)
- `--particle-count <int>` — переопределить количество частиц
- `--dt <float>` — шаг интегрирования физики по времени
- `--physics-steps-per-frame <int>` — шагов физики на кадр (только для GUI)
- `--seed <uint>` — зерно генератора случайных чисел

## Формат JSON-конфигурации

Пример полной конфигурации находится в [run_config.json](file:///home/lin/urfu_proj_cpp/examples/run_config.json).

Структура поддерживает частичную инициализацию: все неуказанные поля заполняются значениями по умолчанию.

Для источников поля (`fieldModel.sources`) поддерживаются:
- Точечные источники: задается параметр `center`
- Линейные источники: задаются параметры `segment.start` и `segment.end`
- Масштаб влияния поля задается через параметр `sigma`

### Как задавать стены и зоны

Стены сцены описываются в секции `scene`:

- `scene.corridor.outline` — внешняя граница допустимой области движения (замкнутый контур)
- `scene.obstacles` — массив независимых внутренних препятствий (замкнутые контуры)
- Вершины контуров соединяются последовательно, последняя автоматически замыкается с первой.
- `scene.spawnZone` и `scene.targetZone` — прямоугольники спавна и цели

Пример фрагмента JSON:

```json
"scene": {
  "corridor": {
    "outline": [
      { "x": 100.0, "y": 40.0 },
      { "x": 180.0, "y": 40.0 },
      { "x": 180.0, "y": 100.0 },
      { "x": 100.0, "y": 100.0 }
    ]
  },
  "obstacles": [
    {
      "outline": [
        { "x": 120.0, "y": 60.0 },
        { "x": 140.0, "y": 60.0 },
        { "x": 140.0, "y": 80.0 },
        { "x": 120.0, "y": 80.0 }
      ]
    }
  ],
  "spawnZone": { "x": 100.0, "y": 40.0, "width": 20.0, "height": 20.0 },
  "targetZone": { "x": 160.0, "y": 40.0, "width": 20.0, "height": 20.0 }
}
```

## Текущая конфигурация оптимизации

Настройки оптимизатора задаются в файле [optimizer_config.json](file:///home/lin/urfu_proj_cpp/examples/optimizer_config.json).

### 1. Целевая функция `quality`

Вычисляется следующим образом:

- Если порог в 80% частиц достигнут (`t80` определено):
  $$quality = t80$$

- Если порог в 80% частиц не достигнут:
  $$quality = maxSimulationTime + penaltyWeight \times missingTargetFraction$$

где:
- $missingTargetFraction = \frac{missingTargetHits}{targetThresholdCount}$ — доля недошедших до цели частиц от требуемого количества для достижения порога 80%.
- $penaltyWeight = 1.0$ (задается в [run_config.json](file:///home/lin/urfu_proj_cpp/examples/run_config.json)).

Чем меньше значение `quality`, тем эффективнее считается конфигурация поля.

### 2. Оптимизируемые параметры

В [optimizer_config.json](file:///home/lin/urfu_proj_cpp/examples/optimizer_config.json) оптимизируются параметры `strength` (сила) и `sigma` (радиус влияния) для четырех основных притягивающих источников:

- `fieldModel.sources.sources[0].strength` (диапазон: от -2200.0 до -1600.0)
- `fieldModel.sources.sources[1].strength` (диапазон: от -3000.0 до -2400.0)
- `fieldModel.sources.sources[2].strength` (диапазон: от -3800.0 до -3200.0)
- `fieldModel.sources.sources[3].strength` (диапазон: от -4600.0 до -4000.0)
- `fieldModel.sources.sources[0].sigma` (диапазон: от 35.0 до 55.0)
- `fieldModel.sources.sources[1].sigma` (диапазон: от 30.0 до 50.0)
- `fieldModel.sources.sources[2].sigma` (диапазон: от 30.0 до 50.0)
- `fieldModel.sources.sources[3].sigma` (диапазон: от 35.0 до 55.0)

Это 8-мерное пространство параметров, определяющее прохождение частиц через критические участки (повороты) S-образного коридора.

### 3. Значения `maxSimulationTime` и `particleCount` в overrides

В файле [optimizer_config.json](file:///home/lin/urfu_proj_cpp/examples/optimizer_config.json) переопределены параметры базовой симуляции:

- `maxSimulationTime = 120.0`
- `particleCount = 20`

Снижение числа частиц с 50 до 20 ускоряет вычисление шага градиентного спуска, а увеличенный лимит в 120 секунд дает частицам достаточно времени для успешного прохождения коридора.

### 4. Воспроизводимость

Для стабильного расчета градиентов методом конечных разностей зафиксирован параметр seed в [run_config.json](file:///home/lin/urfu_proj_cpp/examples/run_config.json):
- `seed = 42`

## Что уже есть в метриках (Headless JSON)

Фоновый запуск симуляции выдает структурированные данные о прогоне:

- `success` — флаг успешного завершения программы
- `runConfig` — конфигурация прогона без GUI-only параметров
- `metrics.reachedT80` — булев флаг достижения порога в 80%
- `metrics.t80` — точное время достижения порога (float или null)
- `metrics.uniqueTargetHits` — количество уникальных частиц, добравшихся до финиша
- `metrics.targetThresholdCount` — целевой порог (80% от количества частиц)
- `metrics.missingTargetHits` — число частиц, которых не хватило до порога
- `metrics.missingTargetFraction` — относительная нехватка частиц
- `metrics.quality` — вычисленное значение целевой функции качества
- `stats.reachedTarget` — массив логических значений для каждой частицы (достигла ли цели)
- `stats.firstHitTime` — время первого попадания для каждой частицы (число или null)
- `runtime.simulatedTime` — промоделированное время в секундах
- `runtime.stepsExecuted` — количество выполненных физических шагов

Пример структуры ответа:

```json
{
  "apiVersion": 1,
  "success": true,
  "runConfig": {
    "particleCount": 20,
    "dt": 0.001,
    "maxSimulationTime": 120.0,
    "seed": 42
  },
  "metrics": {
    "reachedT80": true,
    "t80": 18.52,
    "uniqueTargetHits": 17,
    "targetThresholdCount": 16,
    "missingTargetHits": 0,
    "missingTargetFraction": 0.0,
    "quality": 18.52
  },
  "stats": {
    "reachedTarget": [true, true, false],
    "firstHitTime": [12.4, 18.52, null]
  },
  "runtime": {
    "simulatedTime": 120.0,
    "stepsExecuted": 120000
  }
}
```

## Python-оптимизатор

Скрипт [gradient_descent.py](file:///home/lin/urfu_proj_cpp/scripts/gradient_descent.py):

1. Загружает [optimizer_config.json](file:///home/lin/urfu_proj_cpp/examples/optimizer_config.json).
2. Подгружает базовый [run_config.json](file:///home/lin/urfu_proj_cpp/examples/run_config.json).
3. Применяет overrides (maxSimulationTime, particleCount).
4. Оценивает градиент quality по 8 настраиваемым параметрам.
5. Делает шаг оптимизации, зажимая значения параметров в разрешенных границах.
6. Экспортирует результаты в `runs/gradient_descent/`:
   - `history.csv` — лог всех итераций (quality, status, значения параметров и градиентов)
   - `best_run_config.json` — лучшая конфигурация для запуска в GUI
   - `best_result.json` — подробный headless JSON-результат лучшего прогона
   - `summary.json` — краткая сводка по оптимизации

## Полезные ссылки на файлы

- [main.cpp](file:///home/lin/urfu_proj_cpp/main.cpp) — точка входа, разбор аргументов CLI
- [Config.h](file:///home/lin/urfu_proj_cpp/include/Config.h) — статические настройки (LJ, радиус, скорость)
- [RunConfig.h](file:///home/lin/urfu_proj_cpp/include/RunConfig.h) — конфигурация симуляции
- [SimulationCore.h](file:///home/lin/urfu_proj_cpp/include/SimulationCore.h) — физический движок
- [SimulationStats.h](file:///home/lin/urfu_proj_cpp/include/SimulationStats.h) — вычисление t80 и quality
- [JsonRunIO.cpp](file:///home/lin/urfu_proj_cpp/src/JsonRunIO.cpp) — загрузка/вывод JSON
- [gradient_descent.py](file:///home/lin/urfu_proj_cpp/scripts/gradient_descent.py) — оптимизатор
- [potentials.md](file:///home/lin/urfu_proj_cpp/docs/potentials.md) — физические формулы
