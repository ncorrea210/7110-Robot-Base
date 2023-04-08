#pragma GCC diagnostic ignored "-Wreturn-type"

#include "utils/Subsystem.h"

using namespace hb;

Subsystem::Subsystem() = default;

std::unordered_map<std::string, double> Subsystem::GetTelemetry() {}

void Subsystem::UpdateTelemetry() {}
