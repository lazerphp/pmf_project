#ifndef JSON_RUN_IO_H
#define JSON_RUN_IO_H

#include <string>

#include "HeadlessSimulation.h"
#include "RunConfig.h"

namespace JsonRunIO
{
constexpr int API_VERSION = 1;

RunConfig loadRunConfigFromJsonString(const std::string &jsonText, const RunConfig &defaults = RunConfig{});
RunConfig loadRunConfigFromJsonFile(const std::string &path, const RunConfig &defaults = RunConfig{});

std::string serializeHeadlessResultJson(const HeadlessRunResult &result);
void writeHeadlessResultJsonFile(const HeadlessRunResult &result, const std::string &path);
} // namespace JsonRunIO

#endif // JSON_RUN_IO_H
