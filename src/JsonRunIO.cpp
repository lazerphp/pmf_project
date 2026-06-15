#include "JsonRunIO.h"

#include <fstream>
#include <sstream>
#include <stdexcept>

#include <nlohmann/json.hpp>

using nlohmann::json;

void to_json(json &j, const Vector2 &value)
{
    j = json{{"x", value.x}, {"y", value.y}};
}

void from_json(const json &j, Vector2 &value)
{
    value.x = j.at("x").get<float>();
    value.y = j.at("y").get<float>();
}

void to_json(json &j, const Rect &value)
{
    j = json{
        {"x", value.x},
        {"y", value.y},
        {"width", value.width},
        {"height", value.height},
    };
}

void from_json(const json &j, Rect &value)
{
    value.x = j.at("x").get<float>();
    value.y = j.at("y").get<float>();
    value.width = j.at("width").get<float>();
    value.height = j.at("height").get<float>();
}

namespace Config::Field
{
void to_json(json &j, const Source &source)
{
    j = json{
        {"center", source.center},
        {"strength", source.strength},
        {"sigma", source.sigma},
        {"segment", nullptr},
    };

    if (source.segmentStart.has_value() && source.segmentEnd.has_value())
    {
        j["segment"] = json{
            {"start", *source.segmentStart},
            {"end", *source.segmentEnd},
        };
    }
}

void from_json(const json &j, Source &source)
{
    source.strength = j.at("strength").get<float>();
    source.sigma = j.at("sigma").get<float>();
    source.segmentStart.reset();
    source.segmentEnd.reset();
    source.center = j.value("center", Vector2(0.0f, 0.0f));

    if (j.contains("segment") && !j.at("segment").is_null())
    {
        const json &segment = j.at("segment");
        const Vector2 start = segment.at("start").get<Vector2>();
        const Vector2 end = segment.at("end").get<Vector2>();
        source.segmentStart = start;
        source.segmentEnd = end;
        source.center = Vector2((start.x + end.x) * 0.5f, (start.y + end.y) * 0.5f);
    }
}

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(SourcesParams, sources)

void to_json(json &j, const ModelConfig &config)
{
    j = json{
        {"sources", config.sources},
    };
}

void from_json(const json &j, ModelConfig &config)
{
    config = ModelConfig{};

    if (j.contains("sources"))
    {
        config.sources = j.at("sources").get<SourcesParams>();
    }
}
} // namespace Config::Field

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(QualityConfig, mode, penaltyWeight)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(PolygonConfig, outline)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(SceneConfig, corridor, obstacles, spawnZone, targetZone)

void to_json(json &j, const RunConfig &config)
{
    j = json{
        {"particleCount", config.particleCount},
        {"physicsStepsPerFrame", config.physicsStepsPerFrame},
        {"dt", config.dt},
        {"maxSimulationTime", config.maxSimulationTime},
        {"seed", config.seed},
        {"fieldModel", config.fieldModel},
        {"scene", config.scene},
        {"quality", config.quality},
    };
}

void from_json(const json &j, RunConfig &config)
{
    config = RunConfig{};
    config.particleCount = j.at("particleCount").get<int>();
    config.physicsStepsPerFrame = j.at("physicsStepsPerFrame").get<int>();
    config.dt = j.at("dt").get<float>();
    config.maxSimulationTime = j.at("maxSimulationTime").get<float>();

    if (j.contains("seed") && !j.at("seed").is_null())
    {
        config.seed = j.at("seed").get<unsigned int>();
    }
    else
    {
        config.seed.reset();
    }

    if (j.contains("fieldModel"))
    {
        config.fieldModel = j.at("fieldModel").get<Config::Field::ModelConfig>();
    }

    if (j.contains("scene"))
    {
        config.scene = j.at("scene").get<SceneConfig>();
    }

    config.quality = j.at("quality").get<QualityConfig>();
}

namespace JsonRunIO
{
namespace
{
    bool isSupportedQualityMode(const std::string &mode)
    {
        return mode == "t80_or_penalized_missing_fraction";
    }

    std::string readFile(const std::string &path)
    {
        std::ifstream input(path);
        if (!input)
        {
            throw std::runtime_error("Failed to open JSON input file: " + path);
        }

        std::ostringstream buffer;
        buffer << input.rdbuf();
        return buffer.str();
    }

    void writeFile(const std::string &path, const std::string &content)
    {
        std::ofstream output(path);
        if (!output)
        {
            throw std::runtime_error("Failed to open JSON output file: " + path);
        }

        output << content;
        if (!output)
        {
            throw std::runtime_error("Failed to write JSON output file: " + path);
        }
    }

    void rejectUnknownKeys(const json &patch, const json &schema, const std::string &context)
    {
        if (!patch.is_object() || !schema.is_object())
        {
            return;
        }

        for (auto iterator = patch.begin(); iterator != patch.end(); ++iterator)
        {
            if (!schema.contains(iterator.key()))
            {
                throw std::runtime_error("Unknown key in " + context + ": " + iterator.key());
            }

            const json &schemaValue = schema.at(iterator.key());
            if (iterator->is_object() && schemaValue.is_object())
            {
                rejectUnknownKeys(*iterator, schemaValue, context + "." + iterator.key());
            }
        }
    }

    void mergeJsonPatch(json &base, const json &patch)
    {
        if (!base.is_object() || !patch.is_object())
        {
            base = patch;
            return;
        }

        for (auto iterator = patch.begin(); iterator != patch.end(); ++iterator)
        {
            const std::string &key = iterator.key();
            if (base.contains(key) && base.at(key).is_object() && iterator->is_object())
            {
                mergeJsonPatch(base[key], *iterator);
            }
            else
            {
                base[key] = *iterator;
            }
        }
    }

    json defaultRunConfigJson(const RunConfig &defaults)
    {
        return json{
            {"particleCount", defaults.particleCount},
            {"physicsStepsPerFrame", defaults.physicsStepsPerFrame},
            {"dt", defaults.dt},
            {"maxSimulationTime", defaults.maxSimulationTime},
            {"seed", defaults.seed},
            {"fieldModel", defaults.fieldModel},
            {"scene", defaults.scene},
            {"quality", defaults.quality},
        };
    }
} // namespace

RunConfig loadRunConfigFromJsonString(const std::string &jsonText, const RunConfig &defaults)
{
    const json root = json::parse(jsonText);

    if (!root.is_object())
    {
        throw std::runtime_error("JSON root must be an object");
    }

    if (root.contains("apiVersion"))
    {
        const int apiVersion = root.at("apiVersion").get<int>();
        if (apiVersion != API_VERSION)
        {
            throw std::runtime_error("Unsupported apiVersion: " + std::to_string(apiVersion));
        }
    }

    json mergedRunConfig = defaultRunConfigJson(defaults);

    if (root.contains("runConfig"))
    {
        rejectUnknownKeys(root, json{{"apiVersion", 0}, {"runConfig", mergedRunConfig}}, "root");
        mergeJsonPatch(mergedRunConfig, root.at("runConfig"));
        RunConfig runConfig = mergedRunConfig.get<RunConfig>();
        if (!isSupportedQualityMode(runConfig.quality.mode))
        {
            throw std::runtime_error("Unsupported quality mode: " + runConfig.quality.mode);
        }
        return runConfig;
    }

    json directPatch = root;
    directPatch.erase("apiVersion");
    rejectUnknownKeys(directPatch, mergedRunConfig, "runConfig");
    mergeJsonPatch(mergedRunConfig, directPatch);
    RunConfig runConfig = mergedRunConfig.get<RunConfig>();
    if (!isSupportedQualityMode(runConfig.quality.mode))
    {
        throw std::runtime_error("Unsupported quality mode: " + runConfig.quality.mode);
    }
    return runConfig;
}

RunConfig loadRunConfigFromJsonFile(const std::string &path, const RunConfig &defaults)
{
    return loadRunConfigFromJsonString(readFile(path), defaults);
}

std::string serializeHeadlessResultJson(const HeadlessRunResult &result)
{
    json reachedTarget = json::array();
    for (bool value : result.stats.reachedTarget)
    {
        reachedTarget.push_back(value);
    }

    json firstHitTime = json::array();
    for (const auto &value : result.stats.firstHitTime)
    {
        if (value.has_value())
        {
            firstHitTime.push_back(*value);
        }
        else
        {
            firstHitTime.push_back(nullptr);
        }
    }

    json echoedRunConfig = json(result.runConfig);
    echoedRunConfig.erase("physicsStepsPerFrame");

    json output{
        {"apiVersion", API_VERSION},
        {"success", true},
        {"runConfig", std::move(echoedRunConfig)},
        {"metrics",
         {
             {"reachedT80", result.stats.t80.has_value()},
             {"t80", result.stats.t80.has_value() ? json(*result.stats.t80) : json(nullptr)},
             {"uniqueTargetHits", result.stats.uniqueTargetHits},
             {"targetThresholdCount", result.stats.targetThresholdCount()},
             {"missingTargetHits", result.stats.missingTargetHits()},
             {"missingTargetFraction", result.stats.missingTargetFraction()},
             {"quality", result.stats.computeQuality(result.runConfig.maxSimulationTime, result.runConfig.quality.penaltyWeight)},
         }},
        {"stats",
         {
             {"reachedTarget", std::move(reachedTarget)},
             {"firstHitTime", std::move(firstHitTime)},
         }},
        {"runtime",
         {
             {"simulatedTime", result.simulatedTime},
             {"stepsExecuted", result.stepsExecuted},
         }},
    };

    return output.dump(2);
}

void writeHeadlessResultJsonFile(const HeadlessRunResult &result, const std::string &path)
{
    writeFile(path, serializeHeadlessResultJson(result));
}
} // namespace JsonRunIO
