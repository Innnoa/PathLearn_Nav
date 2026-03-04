#pragma once

#include <algorithm>
#include <cctype>
#include <fstream>
#include <sstream>
#include <string>
#include <unordered_map>

namespace pathlearn::config {

using ConfigMap = std::unordered_map<std::string, std::string>;

enum class LoadStatus {
  kOk = 0,
  kNotFound,
  kIoError,
  kInvalidFormat,
};

struct LoadResult {
  LoadStatus status = LoadStatus::kOk;
  int line = 0;
  std::string message;
};

inline std::string Trim(const std::string& input) {
  size_t left = 0;
  while (left < input.size() &&
         std::isspace(static_cast<unsigned char>(input[left])) != 0) {
    left += 1;
  }
  size_t right = input.size();
  while (right > left &&
         std::isspace(static_cast<unsigned char>(input[right - 1])) != 0) {
    right -= 1;
  }
  return input.substr(left, right - left);
}

inline std::string NormalizeKey(std::string key) {
  key = Trim(key);
  for (char& ch : key) {
    if (ch == '-') {
      ch = '_';
      continue;
    }
    ch = static_cast<char>(
        std::tolower(static_cast<unsigned char>(ch)));
  }
  return key;
}

inline LoadResult LoadConfigFile(
    const std::string& path,
    ConfigMap* output) {
  if (!output) {
    return {LoadStatus::kIoError, 0, "输出映射为空"};
  }
  output->clear();

  std::ifstream input(path);
  if (!input.is_open()) {
    return {LoadStatus::kNotFound, 0, "配置文件不存在"};
  }

  std::string line;
  int line_number = 0;
  while (std::getline(input, line)) {
    line_number += 1;
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }
    std::string trimmed = Trim(line);
    if (trimmed.empty()) {
      continue;
    }
    if (trimmed.rfind("#", 0) == 0 ||
        trimmed.rfind(";", 0) == 0 ||
        trimmed.rfind("//", 0) == 0) {
      continue;
    }

    const size_t equal_pos = trimmed.find('=');
    if (equal_pos == std::string::npos) {
      return {
          LoadStatus::kInvalidFormat,
          line_number,
          "配置行缺少 = 分隔符"};
    }
    std::string key = NormalizeKey(trimmed.substr(0, equal_pos));
    std::string value = Trim(trimmed.substr(equal_pos + 1));
    if (key.empty()) {
      return {
          LoadStatus::kInvalidFormat,
          line_number,
          "配置键为空"};
    }
    if (value.size() >= 2) {
      const char first = value.front();
      const char last = value.back();
      if ((first == '"' && last == '"') ||
          (first == '\'' && last == '\'')) {
        value = value.substr(1, value.size() - 2);
      }
    }
    (*output)[key] = value;
  }

  if (input.bad()) {
    return {LoadStatus::kIoError, 0, "读取配置文件失败"};
  }
  return {LoadStatus::kOk, 0, ""};
}

inline bool HasKey(const ConfigMap& config, const std::string& key) {
  const std::string normalized = NormalizeKey(key);
  return config.find(normalized) != config.end();
}

inline bool TryGetString(
    const ConfigMap& config,
    const std::string& key,
    std::string* value) {
  if (!value) {
    return false;
  }
  const std::string normalized = NormalizeKey(key);
  const auto it = config.find(normalized);
  if (it == config.end()) {
    return false;
  }
  *value = it->second;
  return true;
}

inline bool ParseBool(const std::string& value, bool* output) {
  if (!output) {
    return false;
  }
  std::string normalized;
  normalized.reserve(value.size());
  for (char ch : value) {
    normalized.push_back(static_cast<char>(std::tolower(
        static_cast<unsigned char>(ch))));
  }

  if (normalized == "1" ||
      normalized == "true" ||
      normalized == "on" ||
      normalized == "yes") {
    *output = true;
    return true;
  }
  if (normalized == "0" ||
      normalized == "false" ||
      normalized == "off" ||
      normalized == "no") {
    *output = false;
    return true;
  }
  return false;
}

inline bool TryGetBool(
    const ConfigMap& config,
    const std::string& key,
    bool* value) {
  std::string raw;
  if (!TryGetString(config, key, &raw)) {
    return false;
  }
  return ParseBool(raw, value);
}

inline bool TryGetInt(
    const ConfigMap& config,
    const std::string& key,
    int* value) {
  if (!value) {
    return false;
  }
  std::string raw;
  if (!TryGetString(config, key, &raw)) {
    return false;
  }
  try {
    *value = std::stoi(raw);
  } catch (const std::exception&) {
    return false;
  }
  return true;
}

inline bool TryGetDouble(
    const ConfigMap& config,
    const std::string& key,
    double* value) {
  if (!value) {
    return false;
  }
  std::string raw;
  if (!TryGetString(config, key, &raw)) {
    return false;
  }
  try {
    *value = std::stod(raw);
  } catch (const std::exception&) {
    return false;
  }
  return true;
}

}  // namespace pathlearn::config
