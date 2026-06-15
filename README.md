# Проект: Прохождение частиц через S-образный коридор

## О проекте

Проект моделирует движение частиц внутри S-образного коридора.
Частицы:

- взаимодействуют друг с другом через потенциал Леннарда-Джонса
- движутся под действием внешнего поля
- стартуют из `SpawnZone`
- должны попасть в `TargetZone`

Практическая цель проекта:

- уметь запускать симуляцию интерактивно в окне SFML
- уметь запускать ту же физику в `headless`-режиме
- подбирать параметры внешнего поля автоматически из Python

Сейчас архитектура уже разделена на физическое ядро и GUI-оболочку, поэтому один и тот же набор параметров можно:

- прогонять автоматически без окна
- затем открыть визуально и посмотреть поведение частиц

## Текущее состояние

Уже есть:

- физическое ядро `SimulationCore`
- GUI-оболочка `Simulation`
- headless-режим `./simulation --headless`
- `RunConfig` для runtime-параметров
- загрузка параметров из JSON
- JSON-результат headless-прогона
- базовая метрика `quality` для сравнения прогонов
- отдельный JSON-конфиг для оптимизации
- Python-скрипт для численного градиентного спуска

Основной сценарий работы сейчас такой:

1. Собрать один бинарник `simulation`
2. Запускать его либо в GUI, либо в `--headless`
3. Запускать Python-оптимизатор, который многократно дергает `simulation --headless`
4. Сохранять лучший `RunConfig`
5. При необходимости запускать GUI с тем же конфигом

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

### Ядро

- `SimulationCore` содержит частицы, коридор, препятствия, зоны, статистику и физический шаг
- `Particle`, `Corridor`, `Zone`, `PhysicsForces`, `SimulationStats` не зависят от SFML

### GUI

- `Simulation` отвечает за окно, события, камеру и отрисовку
- `CameraController` и `VisualConfig` относятся только к GUI-слою

### Оптимизатор

- `RunConfig` описывает параметры одного прогона
- `JsonRunIO` загружает конфиги из JSON и сериализует результат
- `HeadlessSimulationRunner` выполняет прогон без окна, возвращает метрики

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
make gui
```

Или всё сразу после сборки:

```bash
make all   # optimize → plot → gui
```

Список всех целей:

```bash
make help
```

## Сборка

### Требования

- C++17-компилятор
- CMake
- SFML 3
- Python 3 + [uv](https://github.com/astral-sh/uv)

### Linux / WSL

```bash
cmake -S . -B build
cmake --build build
```

Если CMake не находит SFML автоматически:

```bash
cmake -S . -B build -DSFML_DIR=/path/to/SFML/lib/cmake/SFML
cmake --build build
```

### Windows / Visual Studio

1. Установить SFML 3 под нужный MSVC
2. Открыть корень проекта в Visual Studio как CMake-проект
3. При необходимости передать:

```json
"cmakeCommandArgs": "-DSFML_DIR=C:/Libs/SFML/lib/cmake/SFML"
```

## Запуск

### GUI

```bash
./build/simulation
```

Без `--input-json` бинарник берет базовую сцену и поле из `examples/run_config.json`.

Во время GUI-прогона:

- `Left` замедляет симуляцию
- `Right` ускоряет симуляцию
- `P` ставит GUI-прогон на паузу и снимает с паузы
- `Space` перезапускает симуляцию

Текущая GUI-скорость показывается в заголовке окна как число `dt`-шагов на кадр.

### GUI с JSON-конфигом

```bash
./build/simulation --input-json examples/run_config.json
```

## Runtime-параметры

Бинарник поддерживает:

- `--input-json <path>`
- `--output-json <path>`
- `--headless`
- `--max-time <float>`
- `--particle-count <int>`
- `--dt <float>`
- `--physics-steps-per-frame <int>` — только для GUI, headless его игнорирует
- `--seed <uint>`

Для `sources` основная настройка поля задается через JSON-конфиг.
Геометрия коридора, внутренних препятствий, стартовой зоны и целевой зоны тоже задается только через JSON.

## Формат JSON

Пример лежит в [examples/run_config.json](/home/lin/urfu_proj_cpp/examples/run_config.json:1).

Общая форма:

```json
{
  "apiVersion": 1,
  "runConfig": {
    "particleCount": 50,
    "physicsStepsPerFrame": 5,
    "dt": 0.001,
    "maxSimulationTime": 10.0,
    "seed": 42,
    "fieldModel": {
      "sources": {
        "sources": [
          {
            "segment": {
              "start": { "x": 560.0, "y": 100.0 },
              "end": { "x": 560.0, "y": 160.0 }
            },
            "strength": -1800.0,
            "sigma": 50.0
          },
          {
            "segment": {
              "start": { "x": 500.0, "y": 360.0 },
              "end": { "x": 560.0, "y": 360.0 }
            },
            "strength": -2500.0,
            "sigma": 45.0
          },
          {
            "segment": {
              "start": { "x": 300.0, "y": 300.0 },
              "end": { "x": 300.0, "y": 360.0 }
            },
            "strength": -3200.0,
            "sigma": 45.0
          },
          {
            "center": { "x": 340.0, "y": 500.0 },
            "strength": -4200.0,
            "sigma": 50.0
          }
        ]
      }
    },
    "scene": {
      "corridor": {
        "outline": [
          { "x": 100.0, "y": 40.0 },
          { "x": 180.0, "y": 40.0 },
          { "x": 180.0, "y": 100.0 }
        ]
      },
      "obstacles": [
        {
          "outline": [
            { "x": 260.0, "y": 180.0 },
            { "x": 300.0, "y": 180.0 },
            { "x": 300.0, "y": 220.0 },
            { "x": 260.0, "y": 220.0 }
          ]
        }
      ],
      "spawnZone": {
        "x": 100.0,
        "y": 40.0,
        "width": 80.0,
        "height": 180.0
      },
      "targetZone": {
        "x": 240.0,
        "y": 460.0,
        "width": 180.0,
        "height": 180.0
      }
    },
    "quality": {
      "mode": "t80_or_penalized_missing_fraction",
      "penaltyWeight": 1.0
    }
  }
}
```

JSON может быть частичным:

- неуказанные поля берутся из `examples/run_config.json`
- это удобно для Python-поиска и ручных экспериментов

Для источников поля можно задавать либо точку, либо отрезок:

- если указан `center`, получится обычный точечный источник
- если указан `segment.start` и `segment.end`, получится линейный источник вдоль конечного отрезка
- `sigma` по-прежнему задает поперечную ширину влияния поля

## Как задавать стены

Стены сцены описываются в секции `scene`.

- `scene.corridor.outline` задает внешнюю границу допустимой области движения.
- `scene.obstacles` задает независимые внутренние препятствия.
- Каждый `outline` задается списком вершин многоугольника.
- Последняя вершина автоматически соединяется с первой, то есть и коридор, и препятствия всегда трактуются как замкнутые контуры.
- Если внутренних препятствий нет, указывай пустой массив: `"obstacles": []`.

Пример:

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

Смысл такой:

- частица должна оставаться внутри `corridor`
- частица не должна попадать внутрь ни одного объекта из `obstacles`

## Конфиг для оптимизации

Чтобы не дублировать геометрию и физику в двух местах, `examples/run_config.json` считается каноническим конфигом симуляции.
`examples/optimizer_config.json` хранит только настройки оптимизации и ссылается на базовый run config.

То есть:

- вся сцена, поле, `quality` и runtime-параметры живут в `run_config.json`
- optimizer хранит `runConfigPath` и, при необходимости, точечные overrides
- список `optimizationParameters` остается рядом с настройками gradient descent

Пример такого файла:

- [examples/optimizer_config.json](/home/lin/urfu_proj_cpp/examples/optimizer_config.json:1)

### Что лежит в optimizer config

`optimizer`
- настройки самого метода оптимизации
- например `iterations`, `learningRate`, `finiteDifferenceStep`

`runConfigPath`
- путь к базовому конфигу симуляции
- обычно это `run_config.json` рядом с optimizer config

`runConfig`
- необязательные overrides поверх базового run config
- удобно, если для оптимизации нужен, например, другой `particleCount` или `maxSimulationTime`

`optimizationParameters`
- список параметров поля, которые разрешено менять
- для каждого параметра хранятся `name`, `min`, `max`

## Текущая конфигурация оптимизации

Ниже описано текущее рабочее состояние optimizer config в репозитории.

### 1. Целевая функция `quality`

Используем:

- если `t80` достигнуто:

```text
quality = t80
```

- если `t80` не достигнуто:

```text
quality = maxSimulationTime + penaltyWeight * missingTargetFraction
```

где:

```text
missingTargetFraction = missingTargetHits / targetThresholdCount
```

Сейчас в базовом `run_config.json`:

- `penaltyWeight = 1.0`

Сейчас поддерживается только один режим `quality`:

- `t80_or_penalized_missing_fraction`

И C++, и Python ожидают именно его.

Почему именно так:

- это одно число, которое удобно минимизировать
- любой успешный прогон автоматически лучше неуспешного
- среди неуспешных лучше тот, где до цели дошло больше частиц
- формула достаточно простая и подходит для ознакомительного gradient descent

### 2. Какие параметры сейчас оптимизируются

Сейчас в [examples/optimizer_config.json](/home/lin/urfu_proj_cpp/examples/optimizer_config.json:1) оптимизируются параметры модели `sources`:

- силы нескольких ключевых притягивающих центров вдоль траектории

Имена параметров задаются как пути внутри JSON, например:

- `fieldModel.sources.sources[2].strength`
- `fieldModel.sources.sources[7].strength`

Почему выбраны именно они:

- они напрямую влияют на то, как частицы проходят повороты S-образного коридора
- это естественная постановка для superposition-модели: геометрия центров фиксирована, а исследуются их интенсивности
- размерность поиска остается умеренной, поэтому gradient descent еще можно отлаживать без слишком дорогих прогонов

### 3. Значение `maxSimulationTime`

Сейчас в overrides внутри optimizer config:

- `maxSimulationTime = 15.0`

Почему:

- 10 секунд для текущих параметров часто недостаточно, чтобы увидеть полезную динамику
- прогон остается информативным, но все еще не слишком дорогим
- для batch-оптимизации важна предсказуемая стоимость одного запуска
- при такой настройке удобно интерпретировать `quality`:
  успешные прогоны обычно дают `quality < 15`,
  неуспешные дают `quality >= 15`

### 4. Seed для воспроизводимости

Для MVP зафиксировано:

- `seed = 42`

Почему:

- численный gradient descent на шумной симуляции иначе ведет себя нестабильно
- для демонстрации и сравнения результатов воспроизводимость важнее случайного разнообразия

### Почему это сделано так

Идея простая:

- README объясняет решение и показывает пример
- Python читает реальный JSON-конфиг
- сами диапазоны и настройки не размазываются по нескольким местам

То есть “официальные числа” живут не в README, а в конфиге.

### Как это используется сейчас

Скрипт [scripts/gradient_descent.py](/home/lin/urfu_proj_cpp/scripts/gradient_descent.py:1) делает именно это:

1. Читает `examples/optimizer_config.json`
2. Загружает базовый `runConfig` по `runConfigPath`
3. Накладывает на него локальные overrides из `runConfig`, если они есть
4. Меняет только параметры из `optimizationParameters`
5. На каждом шаге запускает `./build/simulation --headless --input-json ...`
6. Считает `quality`
7. Оценивает градиент по конечным разностям
8. Обновляет параметры
9. Сохраняет лучший `RunConfig`
10. Сохраняет артефакты в `runs/gradient_descent/`
11. После этого лучший конфиг можно открыть в GUI:

```bash
./build/simulation --input-json runs/gradient_descent/best_run_config.json
```

## Что уже есть в метриках

Headless JSON сейчас возвращает:

- `success`
- `runConfig` без GUI-only поля `physicsStepsPerFrame`
- `t80`
- `uniqueTargetHits`
- `targetThresholdCount`
- `missingTargetHits`
- `missingTargetFraction`
- `quality`
- `stats.reachedTarget`
- `stats.firstHitTime`
- `runtime.simulatedTime`
- `runtime.stepsExecuted`

Смысл:

- `t80` показывает время достижения порога 80%
- `quality` используется для сравнения прогонов при поиске параметров
- `stats` и `runtime` удобны для отладки, анализа и последующей визуализации

Общая форма headless-ответа выглядит так:

```json
{
  "apiVersion": 1,
  "success": true,
  "runConfig": { "...": "..." },
  "metrics": { "...": "..." },
  "stats": {
    "reachedTarget": [true, false],
    "firstHitTime": [5.27, null]
  },
  "runtime": {
    "simulatedTime": 10.0,
    "stepsExecuted": 10000
  }
}
```

### Как сейчас считаются метрики

Для каждой частицы симуляция хранит:

- достигала ли она `TargetZone`
- время первого достижения цели

Из этого считаются:

- `uniqueTargetHits`
  Сколько уникальных частиц впервые достигли цели.

- `targetThresholdCount`
  Сколько частиц нужно для достижения порога 80%.
  Сейчас это `ceil(0.8 * particleCount)`.

- `t80`
  Время, когда число уникальных попаданий впервые достигло `targetThresholdCount`.
  Если порог не достигнут, `t80 = null`.

- `missingTargetHits`
  Сколько уникальных попаданий не хватает до порога 80%.

- `missingTargetFraction`
  Нормированная нехватка до порога:

```text
missingTargetFraction = missingTargetHits / targetThresholdCount
```

- `quality`
  Текущее число для сравнения прогонов.

### Как сейчас считается `quality`

Сейчас реализована простая формула:

- если `t80` достигнуто, то

```text
quality = t80
```

- если `t80` не достигнуто, то

```text
quality = maxSimulationTime + penaltyWeight * missingTargetFraction
```

Сейчас `penaltyWeight = 1.0`.

Это значит:

- успешные прогоны сравниваются по времени достижения 80%
- неуспешные прогоны получают штраф выше `maxSimulationTime`
- чем меньше `quality`, тем лучше прогон

Важно: это уже рабочая формула, но она пока считается базовой и еще может быть уточнена.

## Текущий MVP

Текущий MVP проекта уже включает:

- один бинарник `simulation` с GUI и `--headless` режимом
- единое физическое ядро `SimulationCore`
- загрузку `RunConfig` из JSON
- headless JSON-результат с метриками
- базовую `quality` для сравнения прогонов
- единый optimizer config
- Python-скрипт численного градиентного спуска
- Python-скрипт визуализации результатов оптимизации
- сохранение артефактов оптимизации
- возможность открыть лучший найденный конфиг в GUI

Практически это означает, что уже сейчас можно:

1. Запустить оптимизацию из Python
2. Получить `history.csv`, `best_run_config.json`, `best_result.json`, `summary.json`
3. Открыть лучший конфиг в визуализации через SFML

## Что еще может потребовать подстройки

MVP рабочий, но исследовательская часть проекта, скорее всего, еще потребует итераций.

На практике, возможно, придется дополнительно подбирать:

- окончательную формулу `quality`
- `penaltyWeight`
- набор оптимизируемых параметров
- диапазоны этих параметров
- значение `maxSimulationTime`

Также со временем может понадобиться:

- дополнительная настройка физической модели
- более устойчивый или более быстрый метод оптимизации, если plain gradient descent упрется в локальные минимумы или шум
- более сильные оптимизаторы помимо текущего gradient descent

## Python-оптимизатор

Сейчас базовая версия Python-оптимизатора уже есть:

- [scripts/gradient_descent.py](/home/lin/urfu_proj_cpp/scripts/gradient_descent.py:1)

Она читает optimizer config, запускает headless-прогоны и сохраняет артефакты оптимизации.

### Что принимает `gradient_descent.py`

- `--binary <path>`
  Путь до бинарника `simulation`.
  По умолчанию: `./build/simulation`

- `--optimizer-config <path>`
  Путь до optimizer config.
  По умолчанию: `examples/optimizer_config.json`

- `--output-dir <path>`
  Папка, куда сохраняются результаты.
  По умолчанию: `runs/gradient_descent`

### Что выдает `gradient_descent.py`

Скрипт сохраняет:

- `history.csv`
  История итераций оптимизации.
  Для каждого шага там есть не только `quality`, но и текстовый `status`:
  `reached_t80` или `penalized`.

- `best_run_config.json`
  Лучший найденный конфиг симуляции.

- `best_result.json`
  Полный JSON-результат лучшего headless-прогона.

- `summary.json`
  Короткая сводка по лучшему прогону и настройкам оптимизации.
  Там тоже сохраняется текстовый `bestStatus`.

### Пример запуска

```bash
python3 scripts/gradient_descent.py \
  --binary ./build/simulation \
  --optimizer-config examples/optimizer_config.json \
  --output-dir runs/gradient_descent
```

## Полезные файлы

- [main.cpp](/home/lin/urfu_proj_cpp/main.cpp:1) — CLI, выбор GUI/headless режима
- [include/RunConfig.h](/home/lin/urfu_proj_cpp/include/RunConfig.h:1) — runtime-конфиг прогона
- [include/SimulationCore.h](/home/lin/urfu_proj_cpp/include/SimulationCore.h:1) — физическое ядро
- [include/SimulationStats.h](/home/lin/urfu_proj_cpp/include/SimulationStats.h:1) — статистика и `quality`
- [src/JsonRunIO.cpp](/home/lin/urfu_proj_cpp/src/JsonRunIO.cpp:1) — JSON I/O
- [examples/optimizer_config.json](/home/lin/urfu_proj_cpp/examples/optimizer_config.json:1) — настройки оптимизатора и ссылка на базовый run config
- [scripts/gradient_descent.py](/home/lin/urfu_proj_cpp/scripts/gradient_descent.py:1) — Python-скрипт численного градиентного спуска
- [docs/potentials.md](/home/lin/urfu_proj_cpp/docs/potentials.md:1) — формулы потенциалов
