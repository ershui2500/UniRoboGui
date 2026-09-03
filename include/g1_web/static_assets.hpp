#pragma once

#include <filesystem>
#include <string>

namespace g1_web {

std::string StaticContentType(const std::filesystem::path& path);

bool ResolveStaticAsset(const std::filesystem::path& web_root,
                        const std::string& request_target,
                        std::filesystem::path& resolved);

}  // namespace g1_web
