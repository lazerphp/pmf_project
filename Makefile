BINARY     := ./build/simulation
OPT_CONFIG := examples/optimizer_config.json
RUN_DIR    := runs/gradient_descent
PYTHON     := .venv/bin/python3

.PHONY: build optimize plot plot-save gui gui-best all help

## build       — собрать бинарник
build:
	cmake -S . -B build
	cmake --build build

## optimize    — запустить оптимизацию градиентным спуском
optimize:
	$(PYTHON) scripts/gradient_descent.py \
		--binary $(BINARY) \
		--optimizer-config $(OPT_CONFIG) \
		--output-dir $(RUN_DIR)

## plot        — показать графики по последнему запуску
plot:
	$(PYTHON) scripts/plot_optimization.py --run-dir $(RUN_DIR)

## plot-save   — сохранить графики в PNG
plot-save:
	$(PYTHON) scripts/plot_optimization.py \
		--run-dir $(RUN_DIR) \
		--output $(RUN_DIR)/plot.png

## gui         — запустить GUI с примером актуального конфига
gui:
	$(BINARY) --input-json examples/run_config.json

## gui-best    — запустить GUI с лучшим конфигом из последней оптимизации
gui-best:
	$(BINARY) --input-json $(RUN_DIR)/best_run_config.json

## all         — optimize → plot → gui-best
all: optimize plot gui-best

## help        — список целей
help:
	@grep -E '^##' Makefile | sed 's/## //'
