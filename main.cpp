#include "HeadlessSimulation.h"
#include "JsonRunIO.h"

#ifdef SIMULATION_WITH_GUI
#include "Simulation.h"
#endif

#include <cstdlib>
#include <iostream>
#include <optional>
#include <string>

namespace
{
    const char *DEFAULT_RUN_CONFIG_PATH = "examples/run_config.json";

    void printUsage(const char *programName)
    {
        std::cout << "Usage:\n"
#ifdef SIMULATION_WITH_GUI
                  << "  " << programName << "             Run the GUI simulation\n"
#endif
                  << "  " << programName << " --headless  Run without a window and print JSON metrics\n"
                  << "Common runtime options:\n"
                  << "  --input-json <path>            Load a partial/full run config from JSON\n"
                  << "                                 Default base config: " << DEFAULT_RUN_CONFIG_PATH << "\n"
                  << "  --output-json <path>           Write headless result JSON to a file\n"
                  << "  --particle-count <int>\n"
                  << "  --dt <seconds>\n"
                  << "  --physics-steps-per-frame <int> GUI only\n"
                  << "  --seed <uint>\n"
                  << "Options for --headless:\n"
                  << "  --max-time <seconds>           Stop when simulated time reaches this value\n";
    }

    bool parseFloatArgument(const std::string &value, float &parsedValue)
    {
        char *end = nullptr;
        parsedValue = std::strtof(value.c_str(), &end);
        return end != value.c_str() && end != nullptr && *end == '\0';
    }

    bool parseIntArgument(const std::string &value, int &parsedValue)
    {
        char *end = nullptr;
        const long parsedLong = std::strtol(value.c_str(), &end, 10);
        if (end == value.c_str() || end == nullptr || *end != '\0')
        {
            return false;
        }

        parsedValue = static_cast<int>(parsedLong);
        return true;
    }

    bool parseUnsignedIntArgument(const std::string &value, unsigned int &parsedValue)
    {
        char *end = nullptr;
        const unsigned long parsedLong = std::strtoul(value.c_str(), &end, 10);
        if (end == value.c_str() || end == nullptr || *end != '\0')
        {
            return false;
        }

        parsedValue = static_cast<unsigned int>(parsedLong);
        return true;
    }

    bool parseNextStringArgument(int argc, char **argv, int &index, std::string &parsedValue, const char *optionName)
    {
        if (index + 1 >= argc)
        {
            std::cerr << "Ошибка: после " << optionName << " нужно указать значение.\n";
            return false;
        }

        parsedValue = argv[++index];
        return true;
    }

    bool parseNextFloatArgument(int argc, char **argv, int &index, float &parsedValue, const char *optionName)
    {
        std::string rawValue;
        if (!parseNextStringArgument(argc, argv, index, rawValue, optionName))
        {
            return false;
        }

        if (!parseFloatArgument(rawValue, parsedValue))
        {
            std::cerr << "Ошибка: " << optionName << " должен быть числом.\n";
            return false;
        }

        return true;
    }

    bool parseNextPositiveFloatArgument(int argc, char **argv, int &index, float &parsedValue, const char *optionName)
    {
        if (!parseNextFloatArgument(argc, argv, index, parsedValue, optionName))
        {
            return false;
        }

        if (parsedValue <= 0.0f)
        {
            std::cerr << "Ошибка: " << optionName << " должен быть положительным числом.\n";
            return false;
        }

        return true;
    }

    bool parseNextPositiveIntArgument(int argc, char **argv, int &index, int &parsedValue, const char *optionName)
    {
        std::string rawValue;
        if (!parseNextStringArgument(argc, argv, index, rawValue, optionName))
        {
            return false;
        }

        if (!parseIntArgument(rawValue, parsedValue) || parsedValue <= 0)
        {
            std::cerr << "Ошибка: " << optionName << " должен быть положительным целым числом.\n";
            return false;
        }

        return true;
    }

    bool parseNextUnsignedIntArgument(int argc, char **argv, int &index, unsigned int &parsedValue, const char *optionName)
    {
        std::string rawValue;
        if (!parseNextStringArgument(argc, argv, index, rawValue, optionName))
        {
            return false;
        }

        if (!parseUnsignedIntArgument(rawValue, parsedValue))
        {
            std::cerr << "Ошибка: " << optionName << " должен быть неотрицательным целым числом.\n";
            return false;
        }

        return true;
    }
} // namespace

int main(int argc, char **argv)
{
    try
    {
        bool runHeadless = false;
        bool usedHeadlessOnlyOption = false;
        std::optional<std::string> inputJsonPath;
        std::optional<std::string> outputJsonPath;
        RunConfig runConfig;

        for (int i = 1; i < argc; ++i)
        {
            const std::string argument = argv[i];

            if (argument == "--help")
            {
                printUsage(argv[0]);
                return 0;
            }

            if (argument == "--headless")
            {
                runHeadless = true;
                continue;
            }

            if (argument == "--input-json")
            {
                std::string parsedPath;
                if (!parseNextStringArgument(argc, argv, i, parsedPath, "--input-json"))
                {
                    return -1;
                }

                inputJsonPath = parsedPath;
                continue;
            }

            if (argument == "--output-json")
            {
                std::string parsedPath;
                if (!parseNextStringArgument(argc, argv, i, parsedPath, "--output-json"))
                {
                    return -1;
                }

                usedHeadlessOnlyOption = true;
                outputJsonPath = parsedPath;
            }
        }

        const RunConfig defaultRunConfig = JsonRunIO::loadRunConfigFromJsonFile(DEFAULT_RUN_CONFIG_PATH);
        runConfig = defaultRunConfig;

        if (inputJsonPath.has_value())
        {
            runConfig = JsonRunIO::loadRunConfigFromJsonFile(*inputJsonPath, defaultRunConfig);
        }

        for (int i = 1; i < argc; ++i)
        {
            const std::string argument = argv[i];

            if (argument == "--help" || argument == "--headless")
            {
                continue;
            }

            if (argument == "--input-json" || argument == "--output-json")
            {
                ++i;
                continue;
            }

            if (argument == "--max-time")
            {
                usedHeadlessOnlyOption = true;
                if (!parseNextPositiveFloatArgument(argc, argv, i, runConfig.maxSimulationTime, "--max-time"))
                {
                    return -1;
                }
                continue;
            }

            if (argument == "--particle-count")
            {
                if (!parseNextPositiveIntArgument(argc, argv, i, runConfig.particleCount, "--particle-count"))
                {
                    return -1;
                }
                continue;
            }

            if (argument == "--dt")
            {
                if (!parseNextPositiveFloatArgument(argc, argv, i, runConfig.dt, "--dt"))
                {
                    return -1;
                }
                continue;
            }

            if (argument == "--physics-steps-per-frame")
            {
                if (!parseNextPositiveIntArgument(argc, argv, i, runConfig.physicsStepsPerFrame, "--physics-steps-per-frame"))
                {
                    return -1;
                }
                continue;
            }

            if (argument == "--seed")
            {
                unsigned int parsedSeed = 0;
                if (!parseNextUnsignedIntArgument(argc, argv, i, parsedSeed, "--seed"))
                {
                    return -1;
                }

                runConfig.seed = parsedSeed;
                continue;
            }

            std::cerr << "Ошибка: неизвестный аргумент " << argument << "\n";
            printUsage(argv[0]);
            return -1;
        }

        if (!runHeadless && usedHeadlessOnlyOption)
        {
            std::cerr << "Ошибка: --max-time и --output-json можно использовать только вместе с --headless.\n";
            return -1;
        }

        if (runHeadless)
        {
            HeadlessSimulationRunner runner;
            const HeadlessRunResult result = runner.run(runConfig);
            const std::string outputJson = JsonRunIO::serializeHeadlessResultJson(result);

            if (outputJsonPath.has_value())
            {
                JsonRunIO::writeHeadlessResultJsonFile(result, *outputJsonPath);
            }

            std::cout << outputJson << std::endl;
            return 0;
        }

#ifdef SIMULATION_WITH_GUI
        std::cout << "Запуск симуляции..." << std::endl;
        Simulation simulation(runConfig);
        simulation.run();
#else
        std::cerr << "Ошибка: GUI-режим не собран. Пересоберите проект с -DBUILD_GUI=ON или используйте --headless.\n";
        return -1;
#endif
    }
    catch (const std::exception &e)
    {
        std::cerr << "Ошибка: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}
