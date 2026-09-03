#include "g1_web/static_assets.hpp"

#include <array>
#include <cctype>
#include <system_error>

namespace g1_web {
namespace {

int HexValue(char value) {
  if (value >= '0' && value <= '9') return value - '0';
  value = static_cast<char>(std::tolower(static_cast<unsigned char>(value)));
  if (value >= 'a' && value <= 'f') return value - 'a' + 10;
  return -1;
}

bool DecodePath(const std::string& encoded, std::string& decoded) {
  decoded.clear();
  decoded.reserve(encoded.size());
  for (std::size_t index = 0; index < encoded.size(); ++index) {
    const char value = encoded[index];
    if (value != '%') {
      decoded.push_back(value);
      continue;
    }
    if (index + 2 >= encoded.size()) return false;
    const int high = HexValue(encoded[index + 1]);
    const int low = HexValue(encoded[index + 2]);
    if (high < 0 || low < 0) return false;
    const char decoded_value = static_cast<char>((high << 4) | low);
    if (decoded_value == '\0') return false;
    decoded.push_back(decoded_value);
    index += 2;
  }
  return true;
}

bool IsInside(const std::filesystem::path& root,
              const std::filesystem::path& candidate) {
  auto root_it = root.begin();
  auto candidate_it = candidate.begin();
  while (root_it != root.end()) {
    if (candidate_it == candidate.end() || *root_it != *candidate_it) {
      return false;
    }
    ++root_it;
    ++candidate_it;
  }
  return true;
}

bool IsRootAsset(const std::string& path) {
  static constexpr std::array<const char*, 9> kAllowed{{
      "index.html", "styles.css", "app.js", "i18n.js", "workspace.js",
      "imu-gauges.js", "robot-viewer.js", "perception.js",
      "joint-debug.js",
  }};
  for (const char* allowed : kAllowed) {
    if (path == allowed) return true;
  }
  return false;
}

}  // namespace

std::string StaticContentType(const std::filesystem::path& path) {
  std::string extension = path.extension().string();
  for (char& value : extension) {
    value = static_cast<char>(std::tolower(static_cast<unsigned char>(value)));
  }
  if (extension == ".html") return "text/html; charset=utf-8";
  if (extension == ".css") return "text/css; charset=utf-8";
  if (extension == ".js") return "application/javascript; charset=utf-8";
  if (extension == ".json") return "application/json; charset=utf-8";
  if (extension == ".urdf" || extension == ".xml") {
    return "application/xml; charset=utf-8";
  }
  if (extension == ".stl") return "model/stl";
  if (extension == ".md") return "text/markdown; charset=utf-8";
  if (extension == ".txt") return "text/plain; charset=utf-8";
  return "application/octet-stream";
}

bool ResolveStaticAsset(const std::filesystem::path& web_root,
                        const std::string& request_target,
                        std::filesystem::path& resolved) {
  if (request_target.find_first_of("?#") != std::string::npos) return false;

  std::string decoded;
  if (!DecodePath(request_target, decoded) || decoded.empty() ||
      decoded.front() != '/' || decoded.find('\\') != std::string::npos) {
    return false;
  }

  std::string relative;
  if (decoded == "/") {
    relative = "index.html";
  } else {
    relative = decoded.substr(1);
  }
  if (!IsRootAsset(relative) && relative.rfind("assets/", 0) != 0) {
    return false;
  }

  const std::filesystem::path relative_path(relative);
  if (relative_path.empty() || relative_path.is_absolute()) return false;
  for (const auto& part : relative_path) {
    if (part == "." || part == ".." || part.empty()) return false;
  }

  std::error_code error;
  const auto canonical_root = std::filesystem::weakly_canonical(web_root, error);
  if (error) return false;
  const auto candidate = std::filesystem::weakly_canonical(
      canonical_root / relative_path, error);
  if (error || !IsInside(canonical_root, candidate) ||
      !std::filesystem::is_regular_file(candidate, error) || error) {
    return false;
  }
  resolved = candidate;
  return true;
}

}  // namespace g1_web
